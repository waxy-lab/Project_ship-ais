# 禁用与 Python 3.13 不兼容的 ROS2 pytest 插件
collect_ignore_glob = []

def pytest_configure(config):
    """在配置阶段禁用有问题的插件"""
    try:
        config.pluginmanager.set_blocked("launch-testing")
    except Exception:
        pass
    try:
        config.pluginmanager.set_blocked("launch_testing")
    except Exception:
        pass
