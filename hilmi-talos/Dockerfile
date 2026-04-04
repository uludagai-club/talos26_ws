FROM ros:noetic-ros-base

LABEL maintainer="TALOS Team"
LABEL description="TALOS Otonom Sürüş Kontrol Sistemi"

# Sistem bağımlılıkları
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-can \
    can-utils \
    iproute2 \
    kmod \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# Çalışma dizini
WORKDIR /app

# Python bağımlılıkları
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Dosyaları kopyala
COPY control.py .
COPY entrypoint.sh .
COPY talos_control.md .

# Entrypoint'i çalıştırılabilir yap
RUN chmod +x entrypoint.sh

# Entrypoint
ENTRYPOINT ["/app/entrypoint.sh"]
CMD ["control.py"]
