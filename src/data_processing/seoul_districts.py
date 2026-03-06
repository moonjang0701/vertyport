"""
Seoul District Configuration for V-World DEM Data
서울시 25개 구 좌표 및 설정
"""

# 서울시 25개 구 경계 좌표
SEOUL_DISTRICTS = {
    "강남구": {
        "lat_min": 37.4780, "lat_max": 37.5270,
        "lon_min": 127.0200, "lon_max": 127.0800,
        "center": (37.5172, 127.0473),
        "description": "강남역, 삼성역, 코엑스"
    },
    "강동구": {
        "lat_min": 37.5200, "lat_max": 37.5600,
        "lon_min": 127.1100, "lon_max": 127.1600,
        "center": (37.5301, 127.1238),
        "description": "천호동, 둔촌동"
    },
    "강북구": {
        "lat_min": 37.6200, "lat_max": 37.6600,
        "lon_min": 127.0100, "lon_max": 127.0400,
        "center": (37.6396, 127.0256),
        "description": "수유역, 미아사거리"
    },
    "강서구": {
        "lat_min": 37.5400, "lat_max": 37.5900,
        "lon_min": 126.8100, "lon_max": 126.8800,
        "center": (37.5509, 126.8495),
        "description": "김포공항, 마곡"
    },
    "관악구": {
        "lat_min": 37.4600, "lat_max": 37.4900,
        "lon_min": 126.9300, "lon_max": 126.9800,
        "center": (37.4784, 126.9516),
        "description": "서울대, 신림역"
    },
    "광진구": {
        "lat_min": 37.5300, "lat_max": 37.5600,
        "lon_min": 127.0700, "lon_max": 127.1100,
        "center": (37.5385, 127.0824),
        "description": "건대입구, 구의동"
    },
    "구로구": {
        "lat_min": 37.4800, "lat_max": 37.5200,
        "lon_min": 126.8500, "lon_max": 126.9100,
        "center": (37.4954, 126.8874),
        "description": "구로디지털단지, 신도림"
    },
    "금천구": {
        "lat_min": 37.4400, "lat_max": 37.4800,
        "lon_min": 126.8900, "lon_max": 126.9200,
        "center": (37.4519, 126.9023),
        "description": "가산디지털단지, 독산동"
    },
    "노원구": {
        "lat_min": 37.6300, "lat_max": 37.6700,
        "lon_min": 127.0500, "lon_max": 127.0900,
        "center": (37.6542, 127.0568),
        "description": "상계역, 중계동"
    },
    "도봉구": {
        "lat_min": 37.6500, "lat_max": 37.7000,
        "lon_min": 127.0300, "lon_max": 127.0700,
        "center": (37.6688, 127.0471),
        "description": "쌍문역, 방학동"
    },
    "동대문구": {
        "lat_min": 37.5800, "lat_max": 37.6000,
        "lon_min": 127.0300, "lon_max": 127.0600,
        "center": (37.5744, 127.0399),
        "description": "회기역, 청량리"
    },
    "동작구": {
        "lat_min": 37.4900, "lat_max": 37.5200,
        "lon_min": 126.9400, "lon_max": 126.9800,
        "center": (37.5124, 126.9395),
        "description": "노량진, 사당역"
    },
    "마포구": {
        "lat_min": 37.5400, "lat_max": 37.5800,
        "lon_min": 126.9200, "lon_max": 126.9700,
        "center": (37.5663, 126.9019),
        "description": "홍대, 마포, 상암"
    },
    "서대문구": {
        "lat_min": 37.5700, "lat_max": 37.6000,
        "lon_min": 126.9300, "lon_max": 126.9700,
        "center": (37.5791, 126.9368),
        "description": "신촌, 홍제, 연희동"
    },
    "서초구": {
        "lat_min": 37.4700, "lat_max": 37.5100,
        "lon_min": 126.9900, "lon_max": 127.0400,
        "center": (37.4837, 127.0324),
        "description": "강남역, 교대역, 양재역"
    },
    "성동구": {
        "lat_min": 37.5500, "lat_max": 37.5800,
        "lon_min": 127.0300, "lon_max": 127.0600,
        "center": (37.5634, 127.0371),
        "description": "왕십리, 성수동"
    },
    "성북구": {
        "lat_min": 37.5900, "lat_max": 37.6200,
        "lon_min": 127.0100, "lon_max": 127.0500,
        "center": (37.5894, 127.0167),
        "description": "성신여대, 정릉, 장위동"
    },
    "송파구": {
        "lat_min": 37.4900, "lat_max": 37.5300,
        "lon_min": 127.0800, "lon_max": 127.1400,
        "center": (37.5145, 127.1059),
        "description": "잠실, 송파, 문정"
    },
    "양천구": {
        "lat_min": 37.5100, "lat_max": 37.5400,
        "lon_min": 126.8500, "lon_max": 126.9000,
        "center": (37.5170, 126.8664),
        "description": "목동, 신정동"
    },
    "영등포구": {
        "lat_min": 37.5100, "lat_max": 37.5400,
        "lon_min": 126.8900, "lon_max": 126.9400,
        "center": (37.5263, 126.8962),
        "description": "여의도, 영등포, 문래동"
    },
    "용산구": {
        "lat_min": 37.5200, "lat_max": 37.5500,
        "lon_min": 126.9600, "lon_max": 127.0000,
        "center": (37.5324, 126.9900),
        "description": "이태원, 용산역, 한강진"
    },
    "은평구": {
        "lat_min": 37.5900, "lat_max": 37.6400,
        "lon_min": 126.9100, "lon_max": 126.9600,
        "center": (37.6176, 126.9227),
        "description": "연신내, 불광, 응암"
    },
    "종로구": {
        "lat_min": 37.5700, "lat_max": 37.6000,
        "lon_min": 126.9700, "lon_max": 127.0100,
        "center": (37.5730, 126.9794),
        "description": "광화문, 종로, 인사동"
    },
    "중구": {
        "lat_min": 37.5500, "lat_max": 37.5700,
        "lon_min": 126.9700, "lon_max": 127.0100,
        "center": (37.5636, 126.9976),
        "description": "명동, 을지로, 남대문"
    },
    "중랑구": {
        "lat_min": 37.5900, "lat_max": 37.6200,
        "lon_min": 127.0700, "lon_max": 127.1100,
        "center": (37.6063, 127.0925),
        "description": "면목동, 망우동, 중화동"
    }
}

# 인기 구간 (테스트용)
POPULAR_ROUTES = {
    "강남구_송파구": {
        "origin": "강남구",
        "destination": "송파구",
        "origin_point": (37.4979, 127.0276),  # 강남역
        "dest_point": (37.5126, 127.0588),     # 잠실역
        "distance_km": 3.2,
        "description": "강남역 → 잠실역"
    },
    "여의도_강남": {
        "origin": "영등포구",
        "destination": "강남구",
        "origin_point": (37.5219, 126.9245),  # 여의도
        "dest_point": (37.5172, 127.0473),     # 강남
        "distance_km": 9.8,
        "description": "여의도 → 강남"
    },
    "시청_강남": {
        "origin": "중구",
        "destination": "강남구",
        "origin_point": (37.5665, 126.9780),  # 시청
        "dest_point": (37.5172, 127.0473),     # 강남
        "distance_km": 7.5,
        "description": "시청 → 강남"
    },
    "김포공항_강남": {
        "origin": "강서구",
        "destination": "강남구",
        "origin_point": (37.5632, 126.8014),  # 김포공항
        "dest_point": (37.5172, 127.0473),     # 강남
        "distance_km": 23.5,
        "description": "김포공항 → 강남"
    }
}

def get_district_info(district_name: str) -> dict:
    """
    Get district information
    
    Args:
        district_name: 구 이름 (예: "강남구")
    
    Returns:
        District information dictionary
    """
    if district_name not in SEOUL_DISTRICTS:
        raise ValueError(f"Invalid district name: {district_name}. Available: {list(SEOUL_DISTRICTS.keys())}")
    
    return SEOUL_DISTRICTS[district_name]

def list_districts() -> list:
    """Get list of all Seoul districts"""
    return sorted(SEOUL_DISTRICTS.keys())

def get_route_info(route_name: str) -> dict:
    """Get popular route information"""
    if route_name not in POPULAR_ROUTES:
        raise ValueError(f"Invalid route name: {route_name}. Available: {list(POPULAR_ROUTES.keys())}")
    
    return POPULAR_ROUTES[route_name]
