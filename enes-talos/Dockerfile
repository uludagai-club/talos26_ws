FROM ros:noetic-robot

# Python çıktılarının anlık görünmesi için
ENV PYTHONUNBUFFERED=1

# Gerekli paketlerin kurulumu
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rospy \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Kodun kopyalanması
COPY karar.py /app/karar.py

# Çalıştırma izni
RUN chmod +x /app/karar.py

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python3", "/app/karar.py"]