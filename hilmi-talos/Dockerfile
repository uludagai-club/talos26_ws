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
    && rm -rf /var/lib/apt/lists/*

# Python bağımlılıkları
RUN pip3 install numpy python-can

# Çalışma dizini
WORKDIR /app

# Dosyaları kopyala
COPY control.py .
COPY entrypoint.sh .

# Entrypoint'i çalıştırılabilir yap
RUN chmod +x entrypoint.sh

# Entrypoint
ENTRYPOINT ["/app/entrypoint.sh"]
CMD ["control.py"]
