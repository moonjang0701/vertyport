# 🚁 UAM 버티포트 최적화 시스템 - 설치 가이드

## 📋 시스템 요구사항

- Python 3.8 이상
- Windows 10/11, macOS, Linux

## 🔧 설치 방법

### 1. 저장소 클론

```bash
git clone https://github.com/moonjang0701/vertyport.git
cd vertyport
```

### 2. 가상환경 생성 (권장)

**Windows:**
```bash
python -m venv venv
venv\Scripts\activate
```

**macOS/Linux:**
```bash
python3 -m venv venv
source venv/bin/activate
```

### 3. 필수 패키지 설치

```bash
pip install -r requirements.txt
```

**주의**: scipy 설치 시 오류가 발생하면:
```bash
pip install --upgrade pip setuptools wheel
pip install scipy
```

## ⚠️ 흔한 설치 오류 해결

### 1. `ModuleNotFoundError: No module named 'rasterio'`

**해결책**: rasterio는 GeoTIFF 파일 저장용 선택 패키지입니다. 설치하지 않아도 시스템은 정상 작동합니다.

설치를 원하는 경우:
```bash
pip install rasterio
```

Windows에서 설치 실패 시:
```bash
# Microsoft C++ Build Tools 설치 필요
# 또는 conda 사용
conda install -c conda-forge rasterio
```

### 2. `opencv-python` 설치 오류

```bash
pip install opencv-python-headless
```

### 3. `numba` 설치 오류 (선택사항)

numba는 성능 최적화용이며 필수가 아닙니다. 설치 건너뛰어도 됩니다.

## 🚀 빠른 시작

### 테스트 실행

```bash
python test_vworld_system.py
```

**출력 파일:**
- `vworld_uam_map.html` - 인터랙티브 지도
- `vworld_uam_3d.html` - 3D 시각화

### 웹 인터페이스 실행 (원본 시스템)

```bash
streamlit run app.py
```

브라우저에서 `http://localhost:8501` 접속

## 📦 최소 설치 (핵심 기능만)

빠른 테스트를 위한 최소 패키지:

```bash
pip install numpy scipy matplotlib plotly folium branca pillow
```

이후 필요에 따라 추가 패키지 설치:

```bash
# 웹 인터페이스
pip install streamlit

# 이미지 처리
pip install opencv-python

# GeoTIFF 지원 (선택)
pip install rasterio
```

## 🔍 설치 확인

```python
python -c "import numpy, scipy, plotly, folium; print('설치 성공!')"
```

## 💡 Windows 사용자 팁

### Microsoft C++ Build Tools 필요 시

일부 패키지(scipy, numba)는 C++ 컴파일러가 필요할 수 있습니다.

**해결책 1**: Anaconda 사용
```bash
conda create -n uam python=3.10
conda activate uam
conda install -c conda-forge numpy scipy matplotlib plotly folium
pip install streamlit opencv-python
```

**해결책 2**: Microsoft C++ Build Tools 설치
- https://visualstudio.microsoft.com/downloads/
- "Build Tools for Visual Studio" 다운로드
- "C++ build tools" 워크로드 선택

## 🐛 문제 해결

### ImportError 발생 시

```bash
# 패키지 재설치
pip uninstall <패키지명>
pip install <패키지명>
```

### 버전 충돌 시

```bash
# 가상환경 재생성
deactivate
rm -rf venv  # Windows: rmdir /s venv
python -m venv venv
# 가상환경 활성화 후 재설치
```

## 📞 도움이 필요하신가요?

1. GitHub Issues: https://github.com/moonjang0701/vertyport/issues
2. 오류 메시지와 함께 상세한 환경 정보를 제공해주세요:
   - OS 버전
   - Python 버전 (`python --version`)
   - 설치 시도한 명령어
   - 전체 오류 메시지

## ✅ 다음 단계

설치가 완료되면:

1. **테스트 실행**: `python test_vworld_system.py`
2. **문서 확인**: `ALGORITHM.md` - 알고리즘 상세 설명
3. **시각화 확인**: 생성된 HTML 파일을 브라우저에서 열기

---

**참고**: 이 시스템은 실제 V-World API 키 없이도 합성 데이터로 테스트 가능합니다.
