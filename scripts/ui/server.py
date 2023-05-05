#!/usr/bin/env python3
from flask import Flask, request, Response
import rospy
from std_msgs.msg import String
from flask_cors import CORS, cross_origin
# ROS
rospy.init_node('rest_api', anonymous=True)
pump_pub = rospy.Publisher('PUMP_CTRL', String, queue_size=10)
lift_pub = rospy.Publisher('LIFT_CTRL', String, queue_size=10)
arm_pub = rospy.Publisher('ARM_CTRL', String, queue_size=10)
nav_pub = rospy.Publisher('NAV_CTRL', String, queue_size=10)

busy_status = "BUSY"
free_status = "FREE"

def robot_is_free():
    status = rospy.get_param('/status', free_status)
    if status == free_status:
        return True

    return False

# Flask
serve_host = "0.0.0.0"
serve_port = "8000"

app = Flask(__name__)
cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'

ROBOT_FREE_STATUS = Response(
        response="Waiter_ROS Status: FREE",
        status= 200
    )

ROBOT_BUSY_STATUS = Response(
        response="Robot is not Free",
        status= 400
    )

SUCCESS_STATUS = Response(
        response="Success!",
        status= 200
    )

# Health Checker
@app.route("/")
def home():
    if not robot_is_free():
        return ROBOT_BUSY_STATUS

    return ROBOT_FREE_STATUS

# Robot Arm
@app.route("/operation/arm", methods=["POST"])
def arm_ctrl():
    if not robot_is_free():
        return ROBOT_BUSY_STATUS

    req_data = request.json['data']
    arm_pub.publish(req_data)

    return SUCCESS_STATUS

# Pump
@app.route("/operation/pump", methods=["POST"])
def pump_ctrl():
    if not robot_is_free():
        return Response(
            response="Robot is not Free",
            status= 400
        )

    req_data = request.json['data']
    pump_pub.publish(req_data)

    return SUCCESS_STATUS

# Lift
@app.route("/operation/lift", methods=["POST"])
def lift_ctrl():
    if not robot_is_free():
        return ROBOT_BUSY_STATUS

    req_data = request.json['data']
    lift_pub.publish(req_data)

    return SUCCESS_STATUS

# Navigation
@app.route("/operation/nav", methods=["POST"])
def nav_ctrl():
    if not robot_is_free():
        return ROBOT_BUSY_STATUS

    req_data = request.json['data']
    nav_pub.publish(req_data)

    return SUCCESS_STATUS

if __name__ == '__main__':
    app.run(
        host = serve_host,
        port = serve_port,
        debug = True,
    )