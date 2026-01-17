FROM python:3.9-slim

# Install dependencies
RUN pip install "paho-mqtt<2.0.0" pyserial

# Setup app directory
WORKDIR /app
COPY main.py .

# Run unbuffered (so logs show up instantly)
CMD ["python", "-u", "main.py"]