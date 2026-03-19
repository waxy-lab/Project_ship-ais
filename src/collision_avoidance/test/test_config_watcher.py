"""
配置热加载模块单元测试 Requirements: 8.1
"""
import os
import time
import pytest
from collision_avoidance.config_watcher import ConfigWatcher, MultiConfigWatcher


def write_yaml(path, content):
    with open(path, 'w') as f:
        f.write(content)
    os.utime(path, (time.time() + 0.01, time.time() + 0.01))


@pytest.fixture
def tmp_yaml(tmp_path):
    p = tmp_path / 'config.yaml'
    write_yaml(str(p), 'key1: value1\nkey2: 42\n')
    return str(p)


class TestInit:
    def test_empty_path_raises(self):
        with pytest.raises(ValueError):
            ConfigWatcher('')

    def test_zero_interval_raises(self):
        with pytest.raises(ValueError):
            ConfigWatcher('x.yaml', poll_interval=0.0)

    def test_nonexistent_no_crash(self):
        w = ConfigWatcher('/no/such/file.yaml')
        assert w.error_count >= 1

    def test_initial_load(self, tmp_yaml):
        w = ConfigWatcher(tmp_yaml)
        assert w.reload_count == 1

    def test_config_path_preserved(self, tmp_yaml):
        w = ConfigWatcher(tmp_yaml)
        assert w.config_path == tmp_yaml


class TestRead:
    def test_get_key(self, tmp_yaml):
        assert ConfigWatcher(tmp_yaml).get('key1') == 'value1'

    def test_get_int(self, tmp_yaml):
        assert ConfigWatcher(tmp_yaml).get('key2') == 42

    def test_get_missing_default(self, tmp_yaml):
        assert ConfigWatcher(tmp_yaml).get('x', 'DEF') == 'DEF'

    def test_config_is_dict(self, tmp_yaml):
        assert isinstance(ConfigWatcher(tmp_yaml).config, dict)

    def test_config_returns_copy(self, tmp_yaml):
        w = ConfigWatcher(tmp_yaml)
        assert w.config is not w.config

    def test_nested(self, tmp_path):
        p = tmp_path / 'n.yaml'
        write_yaml(str(p), 'a:\n  b: 99\n')
        assert ConfigWatcher(str(p)).get_nested('a', 'b') == 99

    def test_nested_missing_default(self, tmp_yaml):
        assert ConfigWatcher(tmp_yaml).get_nested('x', 'y', default=42) == 42


class TestReload:
    def test_returns_true(self, tmp_yaml):
        assert ConfigWatcher(tmp_yaml).reload() is True

    def test_updates_config(self, tmp_yaml):
        w = ConfigWatcher(tmp_yaml)
        write_yaml(tmp_yaml, 'key1: new\n')
        w.reload()
        assert w.get('key1') == 'new'

    def test_increments_count(self, tmp_yaml):
        w = ConfigWatcher(tmp_yaml)
        c = w.reload_count
        w.reload()
        assert w.reload_count == c + 1

    def test_nonexistent_returns_false(self):
        w = ConfigWatcher('/no/such/file.yaml')
        assert w.reload() is False


class TestCallbacks:
    def test_cb_on_init(self, tmp_yaml):
        r = []
        ConfigWatcher(tmp_yaml, on_reload=lambda c: r.append(c))
        assert len(r) == 1

    def test_cb_receives_dict(self, tmp_yaml):
        r = []
        ConfigWatcher(tmp_yaml, on_reload=lambda c: r.append(c))
        assert isinstance(r[0], dict)

    def test_cb_on_reload(self, tmp_yaml):
        r = []
        w = ConfigWatcher(tmp_yaml, on_reload=lambda c: r.append(c))
        write_yaml(tmp_yaml, 'key1: v2\n')
        w.reload()
        assert len(r) == 2

    def test_add_cb(self, tmp_yaml):
        r = []
        w = ConfigWatcher(tmp_yaml)
        w.add_callback(lambda c: r.append(c))
        w.reload()
        assert len(r) >= 1

    def test_remove_cb(self, tmp_yaml):
        r = []
        cb = lambda c: r.append(c)
        w = ConfigWatcher(tmp_yaml, on_reload=cb)
        w.remove_callback(cb)
        init = len(r)
        write_yaml(tmp_yaml, 'key1: removed\n')
        w.reload()
        assert len(r) == init

    def test_invalid_cb_raises(self, tmp_yaml):
        with pytest.raises(ValueError):
            ConfigWatcher(tmp_yaml).add_callback('not_callable')


class TestThread:
    def test_not_running_init(self, tmp_yaml):
        assert not ConfigWatcher(tmp_yaml).is_running

    def test_start_stop(self, tmp_yaml):
        w = ConfigWatcher(tmp_yaml, poll_interval=0.1)
        w.start()
        assert w.is_running
        w.stop()
        assert not w.is_running

    def test_auto_start(self, tmp_yaml):
        w = ConfigWatcher(tmp_yaml, poll_interval=0.1, auto_start=True)
        assert w.is_running
        w.stop()

    def test_double_start_safe(self, tmp_yaml):
        w = ConfigWatcher(tmp_yaml, poll_interval=0.1)
        w.start()
        w.start()  # 不应报错
        assert w.is_running
        w.stop()

    def test_hot_reload(self, tmp_yaml):
        r = []
        w = ConfigWatcher(
            tmp_yaml,
            on_reload=lambda c: r.append(c),
            poll_interval=0.1,
            auto_start=True,
        )
        init = len(r)
        write_yaml(tmp_yaml, 'key1: hot\n')
        time.sleep(0.5)
        w.stop()
        assert len(r) > init

    def test_repr(self, tmp_yaml):
        w = ConfigWatcher(tmp_yaml)
        assert 'ConfigWatcher' in repr(w)


class TestMulti:
    def test_empty_raises(self):
        with pytest.raises(ValueError):
            MultiConfigWatcher([])

    def test_merged(self, tmp_path):
        p1 = tmp_path / 'c1.yaml'
        p2 = tmp_path / 'c2.yaml'
        write_yaml(str(p1), 'k1: v1\n')
        write_yaml(str(p2), 'k2: v2\n')
        m = MultiConfigWatcher([str(p1), str(p2)])
        cfg = m.merged_config
        assert 'k1' in cfg and 'k2' in cfg

    def test_later_overrides(self, tmp_path):
        p1 = tmp_path / 'c1.yaml'
        p2 = tmp_path / 'c2.yaml'
        write_yaml(str(p1), 'shared: A\n')
        write_yaml(str(p2), 'shared: B\n')
        assert MultiConfigWatcher([str(p1), str(p2)]).merged_config['shared'] == 'B'

    def test_reload_count(self, tmp_path):
        p1 = tmp_path / 'c1.yaml'
        p2 = tmp_path / 'c2.yaml'
        write_yaml(str(p1), 'k1: v1\n')
        write_yaml(str(p2), 'k2: v2\n')
        mw = MultiConfigWatcher([str(p1), str(p2)])
        assert mw.reload_count >= 2


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
