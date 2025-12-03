import json
import os
from shapely.geometry import shape, Polygon

class MapLoader:
    """
    专门负责地图文件读取和解析的工具类
    """
    
    # 默认的备用围栏 (矩形)，防止文件读取失败导致程序崩溃
    DEFAULT_COORDS = [
        (121.780, 31.380), (121.800, 31.370),
        (121.810, 31.340), (121.775, 31.350),
        (121.780, 31.380)
    ]

    @staticmethod
    def load_from_geojson(file_path, logger=None):
        """
        读取 GeoJSON 并返回 Shapely Polygon 对象
        :param file_path: GeoJSON 文件绝对路径
        :param logger: ROS logger 对象 (用于打印日志)，可选
        :return: shapely.geometry.Polygon
        """
        
        # 内部辅助打印函数
        def log_info(msg):
            if logger: logger.info(f"[MapLoader] {msg}")
            else: print(f"[MapLoader] {msg}")

        def log_error(msg):
            if logger: logger.error(f"[MapLoader] {msg}")
            else: print(f"[Error] {msg}")

        def log_warn(msg):
            if logger: logger.warn(f"[MapLoader] {msg}")
            else: print(f"[Warn] {msg}")

        # --- 开始读取逻辑 ---
        log_info(f"正在加载地图: {file_path}")

        try:
            if not file_path or not os.path.exists(file_path):
                raise FileNotFoundError(f"路径无效或文件不存在: {file_path}")

            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)

            # 解析 GeoJSON
            # 这里假设 GeoJSON 里有一个 FeatureCollection，我们取第一个 Feature 的几何形状
            if 'features' in data and len(data['features']) > 0:
                geometry = data['features'][0]['geometry']
                polygon = shape(geometry)
                
                log_info(f"地图加载成功! 包含 {len(data['features'])} 个要素。")
                return polygon
            else:
                raise ValueError("GeoJSON 格式无法解析或为空")

        except Exception as e:
            log_error(f"地图加载失败: {e}")
            log_warn("已启用默认矩形围栏，请检查文件路径。")
            return Polygon(MapLoader.DEFAULT_COORDS)