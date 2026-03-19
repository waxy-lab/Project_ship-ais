"""
配置热加载模块

监听配置文件变化，自动重新加载参数，无需重启节点。
Requirements: 8.1
"""

import os
import time
import threading
import logging
from typing import Any, Callable, Dict, List, Optional

try:
    import yaml
    _YAML_AVAILABLE = True
except ImportError:
    _YAML_AVAILABLE = False

logger = logging.getLogger(__name__)


class ConfigWatcher:
    """
    配置文件热加载监视器

    监听指定 YAML 配置文件的变化（基于文件修改时间），
    检测到变化时自动重新加载并通知回调函数。
    Requirements: 8.1

    用法：
        watcher = ConfigWatcher('config.yaml', on_reload=my_callback)
        watcher.start()
        # ... 应用运行中 ...
        watcher.stop()
    """

    def __init__(
            self,
            config_path: str,
            on_reload: Optional[Callable[[Dict[str, Any]], None]] = None,
            poll_interval: float = 1.0,
            auto_start: bool = False,
    ):
        """
        初始化配置监视器

        Args:
            config_path: 配置文件路径（YAML）
            on_reload: 配置重载回调，接收新配置字典
            poll_interval: 轮询间隔（秒），默认1秒
            auto_start: 是否自动启动监听线程
        """
        if not isinstance(config_path, str) or not config_path:
            raise ValueError('config_path 不能为空')
        if poll_interval <= 0:
            raise ValueError(f'轮询间隔必须大于0: {poll_interval}')

        self._config_path = config_path
        self._on_reload = on_reload
        self._poll_interval = poll_interval
        self._config: Dict[str, Any] = {}
        self._last_mtime: float = 0.0
        self._reload_count: int = 0
        self._error_count: int = 0
        self._callbacks: List[Callable[[Dict[str, Any]], None]] = []
        if on_reload is not None:
            self._callbacks.append(on_reload)

        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        # 初始加载
        self._load_config()

        if auto_start:
            self.start()

    # ------------------------------------------------------------------
    # 公共 API
    # ------------------------------------------------------------------

    @property
    def config(self) -> Dict[str, Any]:
        """当前配置（线程安全）"""
        with self._lock:
            return dict(self._config)

    @property
    def config_path(self) -> str:
        return self._config_path

    @property
    def reload_count(self) -> int:
        """成功重载次数"""
        return self._reload_count

    @property
    def error_count(self) -> int:
        """加载失败次数"""
        return self._error_count

    @property
    def is_running(self) -> bool:
        """监听线程是否在运行"""
        return self._thread is not None and self._thread.is_alive()

    def get(self, key: str, default: Any = None) -> Any:
        """获取配置项（线程安全）"""
        with self._lock:
            return self._config.get(key, default)

    def get_nested(self, *keys: str, default: Any = None) -> Any:
        """
        获取嵌套配置项（线程安全）

        例: watcher.get_nested('collision', 'threshold', default=0.5)
        """
        with self._lock:
            data = self._config
            for k in keys:
                if not isinstance(data, dict):
                    return default
                data = data.get(k, None)
                if data is None:
                    return default
            return data

    def add_callback(self, cb: Callable[[Dict[str, Any]], None]):
        """添加配置重载回调"""
        if not callable(cb):
            raise ValueError('回调必须是可调用对象')
        self._callbacks.append(cb)

    def remove_callback(self, cb: Callable[[Dict[str, Any]], None]):
        """移除配置重载回调"""
        self._callbacks = [c for c in self._callbacks if c is not cb]

    def reload(self) -> bool:
        """
        手动触发重载

        Returns:
            True 表示成功加载，False 表示失败
        """
        return self._load_config(force=True)

    def start(self):
        """启动后台监听线程"""
        if self.is_running:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._watch_loop,
            name=f'ConfigWatcher-{os.path.basename(self._config_path)}',
            daemon=True,
        )
        self._thread.start()
        logger.info(f'ConfigWatcher 已启动: {self._config_path} '
                    f'(间隔={self._poll_interval}s)')

    def stop(self, timeout: float = 3.0):
        """停止后台监听线程"""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None
        logger.info(f'ConfigWatcher 已停止: {self._config_path}')

    # ------------------------------------------------------------------
    # 内部方法
    # ------------------------------------------------------------------

    def _watch_loop(self):
        """后台轮询循环"""
        while not self._stop_event.is_set():
            self._check_and_reload()
            self._stop_event.wait(self._poll_interval)

    def _check_and_reload(self):
        """检查文件是否变化，若变化则重载"""
        try:
            if not os.path.exists(self._config_path):
                return
            mtime = os.path.getmtime(self._config_path)
            if mtime > self._last_mtime:
                self._load_config()
        except OSError as e:
            logger.warning(f'检查配置文件失败: {e}')

    def _load_config(self, force: bool = False) -> bool:
        """
        加载/重载配置文件

        Returns:
            True 表示成功，False 表示失败
        """
        if not os.path.exists(self._config_path):
            logger.warning(f'配置文件不存在: {self._config_path}')
            self._error_count += 1
            return False

        try:
            mtime = os.path.getmtime(self._config_path)
            if not force and mtime <= self._last_mtime and self._reload_count > 0:
                return True  # 未变化

            if _YAML_AVAILABLE:
                with open(self._config_path, 'r', encoding='utf-8') as f:
                    new_config = yaml.safe_load(f) or {}
            else:
                # fallback: 简单key=value解析
                new_config = self._simple_parse()

            if not isinstance(new_config, dict):
                raise ValueError(f'配置文件根节点必须是字典: {type(new_config)}')

            with self._lock:
                self._config = new_config
                self._last_mtime = mtime

            self._reload_count += 1
            logger.info(f'配置已重载 (第{self._reload_count}次): {self._config_path}')

            # 通知所有回调
            for cb in list(self._callbacks):
                try:
                    cb(dict(new_config))
                except Exception as e:
                    logger.error(f'配置回调执行失败: {e}')

            return True

        except Exception as e:
            self._error_count += 1
            logger.error(f'配置加载失败: {self._config_path}: {e}')
            return False

    def _simple_parse(self) -> Dict[str, Any]:
        """简单 key: value 解析（YAML不可用时的备用方案）"""
        result = {}
        with open(self._config_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                if ':' in line:
                    k, _, v = line.partition(':')
                    result[k.strip()] = v.strip()
        return result

    def __repr__(self) -> str:
        return (
            f'ConfigWatcher(path={self._config_path!r}, '
            f'running={self.is_running}, '
            f'reloads={self._reload_count})'
        )


class MultiConfigWatcher:
    """
    多配置文件监视器

    同时监听多个配置文件，合并为统一配置视图。
    Requirements: 8.1
    """

    def __init__(
            self,
            config_paths: List[str],
            on_reload: Optional[Callable[[Dict[str, Any]], None]] = None,
            poll_interval: float = 1.0,
    ):
        if not config_paths:
            raise ValueError('至少需要一个配置文件路径')
        self._watchers = [
            ConfigWatcher(p, poll_interval=poll_interval)
            for p in config_paths
        ]
        self._on_reload = on_reload
        if on_reload:
            for w in self._watchers:
                w.add_callback(lambda cfg, w=w: self._on_any_change())

    def _on_any_change(self):
        if self._on_reload:
            self._on_reload(self.merged_config)

    @property
    def merged_config(self) -> Dict[str, Any]:
        """合并所有配置文件（后者覆盖前者）"""
        merged: Dict[str, Any] = {}
        for w in self._watchers:
            merged.update(w.config)
        return merged

    def start(self):
        for w in self._watchers:
            w.start()

    def stop(self, timeout: float = 3.0):
        for w in self._watchers:
            w.stop(timeout=timeout)

    @property
    def reload_count(self) -> int:
        return sum(w.reload_count for w in self._watchers)
