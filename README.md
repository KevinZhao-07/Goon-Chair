# 🤖 Smart Chair
### Automating Sensitive Medical Sample Collection with Precision and Comfort

<img width="854" height="697" alt="image.jpg" src="https://github.com/KevinZhao-07/Go-on-Chair/blob/main/test/image.jpg?raw=true"/>

---

## 💡 Inspiration
Our team wanted to explore how **robotics** and **computer vision** can be used to automate delicate clinical or laboratory procedures where **precision**, **safety**, and **privacy** are essential.  
We were inspired by the potential to **reduce human involvement** in repetitive or discomfort-inducing tasks while maintaining **accuracy**, **reliability**, and **user comfort**.

---

## 🧠 What We Learned
Throughout the project, we learned how to:
- 🖥️ Use **OpenCV** for real-time human tracking and object recognition.  
- ⚡ Interface **Arduino microcontrollers** with motor drivers and ultrasonic distance sensors for coordinated movement.  
- 🛠️ Design a **mechanical actuation system** for repeatable, controlled motion.  
- 🔄 Implement **feedback loops** for adaptive and safe robotic behavior.  
- ⚖️ Consider **ethics, safety, and privacy** in human-interactive robotics.

---

## 🏗️ How We Built It

### Hardware Setup
- Connected an **Arduino** to **two 12V DC motors** via a motor driver.  
- Added an **ultrasonic sensor** to detect proximity and user presence.  

### Software & Communication
- Used **Python** with **OpenCV** to implement a **real-time person-following and object-detection system**.  
- Sent commands from Python to Arduino via **serial communication**, synchronizing movement and feedback.  

### Control & Actuation
- Programmed logic for **speed adjustment**, **direction control**, and **presence detection**.  
- Designed **mechanical components** to perform automated actions based on sensor input.

---

## ⚠️ Challenges We Faced
- Maintaining stable **OpenCV tracking** under variable lighting conditions.  
- Synchronizing **real-time vision processing** with hardware actuation.  
- Balancing **torque** and **power constraints** to prevent stalling or overheating.  
- Ensuring **electrical and mechanical safety** during continuous operation.

---

## ✅ Key Takeaway
This project combined **software**, **hardware**, and **design thinking** to demonstrate how robotics can improve **precision**, **automation**, and **safety** in **assistive and clinical environments**.
