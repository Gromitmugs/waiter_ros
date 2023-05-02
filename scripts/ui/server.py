#!/usr/bin/env python3
from flask import Flask, request

serve_host = "0.0.0.0"
serve_port = "8000"

app = Flask(__name__)

# Health Checker
@app.route("/")
def home():
    return "waiter_ros"

# Robot Arm
@app.route("/operation/arm_ctrl", methods=["POST"])
def arm_ctrl():
    req_data = request.json['data']
    return "request:" + req_data

# Pump
@app.route("/operation/pump_ctrl", methods=["POST"])
def arm_ctrl():
    req_data = request.json['data']
    return "request:" + req_data

# Lift
@app.route("/operation/lift_ctrl", methods=["POST"])
def arm_ctrl():
    req_data = request.json['data']
    return "request:" + req_data

# Navigation
@app.route("/operation/nav_ctrl", methods=["POST"])
def arm_ctrl():
    req_data = request.json['data']
    return "request:" + req_data

if __name__ == '__main__':
    app.run(
        host = serve_host,
        port = serve_port,
        debug = True,
    )