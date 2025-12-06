---
sidebar_position: 1
title: ROS 2 کا تعارف
---

import { LanguageSwitcher } from '@site/src/components/LanguageSwitcher';

# ROS 2 Fundamentals

<LanguageSwitcher />

## ROS 2 کیا ہے؟

<span className="technical-term">Robot Operating System 2 (ROS 2)</span> ایک open-source framework ہے جو روبوٹ سافٹ ویئر بنانے کے لیے استعمال ہوتا ہے۔ یہ صنعتی معیار ہے جو دنیا بھر میں ہزاروں روبوٹکس انجینئرز استعمال کرتے ہیں۔

### کلیدی خصوصیات

- **Distributed Architecture**: ایک سے زیادہ processes مواصلات کر سکتی ہیں
- **Real-time Support**: سخت وقت کی حدود کے ساتھ کنٹرول
- **Cross-platform**: Linux، Windows، macOS
- **Language Support**: Python، C++، اور مزید

## AI + Physical Integration

### LLM Agents بطور ROS 2 Nodes

روایتی ROS 2 میں، آپ sensor data کو manually process کرتے ہیں۔ <span className="technical-term">Physical AI</span> کے ساتھ، آپ AI models استعمال کر سکتے ہیں decision making کے لیے:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import openai

class LLMRobotController(Node):
    """
    AI-powered robot controller جو GPT-4 استعمال کرتا ہے
    sensor data کی بنیاد پر فیصلے کرنے کے لیے
    """

    def __init__(self):
        super().__init__('llm_controller')

        # ROS 2 publishers اور subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # OpenAI client
        self.llm_client = openai.OpenAI(api_key="YOUR_API_KEY")

    def scan_callback(self, msg):
        """
        LIDAR data وصول کریں اور AI سے مشورہ لیں
        """
        # سامنے کی رکاوٹ کی دوری حاصل کریں
        front_distance = min(msg.ranges[0:10])

        # AI سے پوچھیں کہ کیا کریں
        prompt = f"""
        Robot sensor رکاوٹ کا پتہ لگاتا ہے {front_distance:.2f}m آگے۔
        کیا مجھے: (A) آگے بڑھنا چاہیے، (B) رکنا چاہیے، یا (C) مڑنا چاہیے؟
        صرف حرف کے ساتھ جواب دیں۔
        """

        response = self.llm_client.chat.completions.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}]
        )

        action = response.choices[0].message.content.strip()

        # AI کے فیصلے کو robot commands میں تبدیل کریں
        cmd = Twist()
        if action == "A":
            cmd.linear.x = 0.5  # آگے بڑھیں
        elif action == "B":
            cmd.linear.x = 0.0  # رکیں
        elif action == "C":
            cmd.angular.z = 0.5  # مڑیں

        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'AI فیصلہ: {action}')
```

**Physical Execution**: AI سوچتا ہے (GPT-4) → ROS 2 حکم بھیجتا ہے → Robot عمل کرتا ہے

## بنیادی تصورات

### 1. Nodes

<span className="technical-term">Node</span> ایک independent process ہے جو ایک مخصوص کام انجام دیتا ہے:

```python
# Node بنائیں
rclpy.init()
node = rclpy.create_node('my_robot_node')

# Node چلائیں
rclpy.spin(node)
```

### 2. Topics

<span className="technical-term">Topics</span> nodes کے درمیان data بھیجنے کا طریقہ ہیں:

```python
# Publisher بنائیں
publisher = node.create_publisher(String, 'robot_status', 10)

# Data publish کریں
msg = String()
msg.data = 'Robot چل رہا ہے'
publisher.publish(msg)
```

### 3. Services

<span className="technical-term">Services</span> request-response مواصلات کے لیے استعمال ہوتے ہیں:

```python
# Service client بنائیں
client = node.create_client(AddTwoInts, 'add_numbers')

# Request بھیجیں
request = AddTwoInts.Request()
request.a = 5
request.b = 3
response = client.call(request)
```

## اگلے قدم

1. [Nodes اور Topics](/docs/module-1-ros2/nodes) میں گہرائی سے جائیں
2. [Services اور Actions](/docs/module-1-ros2/services) سیکھیں
3. [URDF میں Robot Models](/docs/module-1-ros2/urdf) بنائیں

## وسائل

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [OpenAI + ROS 2 Integration Guide](https://github.com/ros-planning/moveit2)

---

**یاد رکھیں**: تمام code examples میں تکنیکی اصطلاحات انگریزی میں ہیں تاکہ آپ official documentation کے ساتھ آسانی سے کام کر سکیں۔
