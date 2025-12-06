---
sidebar_position: 2
title: خوش آمدید
---

import { LanguageSwitcher } from '@site/src/components/LanguageSwitcher';
import { EnvironmentSwitcher } from '@site/src/components/EnvironmentSwitcher';

# Physical AI اور Humanoid Robotics میں خوش آمدید

**محسوس کرنے، سوچنے اور عمل کرنے** والے روبوٹ بنائیں جو حقیقی دنیا میں کام کر سکیں۔

<LanguageSwitcher />

<EnvironmentSwitcher />

## Physical AI کیا ہے؟

<span className="technical-term">Physical AI</span> مصنوعی ذہانت کا اگلا مرحلہ ہے: **embodied intelligence** جو حقیقی دنیا میں محسوس، سوچ اور عمل کر سکتی ہے۔ روایتی AI کے برعکس جو صرف سافٹ ویئر میں موجود ہے، <span className="technical-term">Physical AI</span>:

- **محسوس کرتی ہے** ماحول کو <span className="technical-term">sensors</span> کے ذریعے (cameras، LiDAR، IMUs)
- **سوچتی ہے** AI models استعمال کرتے ہوئے (<span className="technical-term">LLMs</span>، vision-language models، planning algorithms)
- **عمل کرتی ہے** جسمانی <span className="technical-term">actuators</span> کے ذریعے (motors، grippers، mobile bases)

## کورس کا جائزہ

یہ پلیٹ فارم **13 ہفتوں کے نصاب** کے ساتھ چار اہم ماڈیولز پر مشتمل ہے:

### ماڈیول 1: ROS 2 بنیادی باتیں (ہفتے 3-5)

<span className="technical-term">Robot Operating System 2 (ROS 2)</span> سیکھیں - روبوٹ سافٹ ویئر ڈویلپمنٹ کے لیے صنعتی معیار۔ <span className="technical-term">nodes</span>، <span className="technical-term">topics</span>، <span className="technical-term">services</span> میں مہارت حاصل کریں اور Python پر مبنی AI controllers کو جسمانی روبوٹ کے ساتھ کیسے مربوط کریں۔

**AI + Physical Integration مثال**: <span className="technical-term">LLM</span> سے پیدا کردہ حرکت کے احکامات استعمال کرتے ہوئے ایک <span className="technical-term">simulated robot arm</span> کو <span className="technical-term">ROS 2 topics</span> کے ذریعے کنٹرول کریں۔

### ماڈیول 2: Digital Twin (ہفتے 6-7)

<span className="technical-term">Gazebo</span> اور <span className="technical-term">Unity</span> استعمال کرتے ہوئے جسمانی روبوٹس کی virtual replicas بنائیں۔ حقیقی ہارڈ ویئر پر deploy کرنے سے پہلے sensors، physics اور environment interactions کو simulate کرنا سیکھیں۔

**AI + Physical Integration مثال**: <span className="technical-term">Gazebo simulation</span> میں ایک <span className="technical-term">reinforcement learning agent</span> کو تربیت دیں، پھر سیکھی ہوئی <span className="technical-term">policy</span> کو حقیقی روبوٹ پر deploy کریں۔

### ماڈیول 3: NVIDIA Isaac Sim (ہفتے 8-9)

<span className="technical-term">NVIDIA Isaac Sim</span> استعمال کرتے ہوئے حقیقت پسند، جسمانی طور پر درست <span className="technical-term">simulation</span> دریافت کریں۔ <span className="technical-term">USD workflows</span>، <span className="technical-term">PhysX engine</span>، اور vision-based AI کے لیے sim-to-real transfer تکنیک سیکھیں۔

**AI + Physical Integration مثال**: <span className="technical-term">Isaac Sim</span> استعمال کرتے ہوئے ایک <span className="technical-term">vision-language model</span> کے لیے synthetic training data پیدا کریں جو روبوٹ کے manipulation tasks کی رہنمائی کرتا ہے۔

### ماڈیول 4: Vision-Language-Action (VLA) Models (ہفتے 10-13)

بڑے language models کو روبوٹ کی <span className="technical-term">perception</span> اور <span className="technical-term">control</span> کے ساتھ مربوط کریں۔ گفتگو کرنے والے روبوٹ بنائیں جو قدرتی زبان کے احکامات کو سمجھ سکیں اور جسمانی اعمال کے ذریعے ان پر عمل کر سکیں۔

**AI + Physical Integration مثال**: ایک <span className="technical-term">Whisper + GPT-4 + ROS 2 pipeline</span> deploy کریں جو آواز کے احکامات سنتا ہے، task execution کے بارے میں سوچتا ہے، اور pick-and-place operations انجام دینے کے لیے robot arm کو کنٹرول کرتا ہے۔

## سیکھنے کا راستہ

یہ کورس آپ کو یہاں سے لے جائے گا:

1. **بنیادی باتیں** (ہفتے 1-2): Python، Linux، Git
2. **ROS 2 Fundamentals** (ہفتے 3-5): Nodes، topics، services
3. **Simulation** (ہفتے 6-9): Gazebo، Isaac Sim
4. **AI Integration** (ہفتے 10-13): VLA models، voice control

## ہارڈ ویئر کی ضروریات

<EnvironmentSwitcher />

### 💻 Workstation Mode
- **GPU**: RTX 4070 Ti یا اس سے زیادہ
- **RAM**: 32GB تجویز کردہ
- **OS**: Ubuntu 22.04 LTS

### ☁️ Cloud Mode
- کوئی مقامی GPU کی ضرورت نہیں
- <span className="technical-term">Omniverse Cloud</span> access
- مستحکم انٹرنیٹ کنکشن

### 🍎 Mac Mode
- macOS 12+
- <span className="technical-term">PyBullet</span> یا <span className="technical-term">Gazebo</span> متبادل
- Cloud GPU تک رسائی تجویز کردہ

## پلیٹ فارم کی خصوصیات

✅ **AI-Powered Chatbot**: RAG استعمال کرتے ہوئے سیاق و سباق کے سوالات
✅ **Hardware-Aware Content**: آپ کے GPU کے لیے موزوں
✅ **Dual Language**: انگریزی اور اردو
✅ **13-Week Syllabus**: منظم سیکھنے کا راستہ
✅ **Constitutional Compliance**: تمام مواد میں AI + Physical integration

## شروع کریں

1. [سائن اپ کریں](/signup) اپنا hardware profile بنانے کے لیے
2. اپنا ترجیحی زبان اور environment mode منتخب کریں
3. [ماڈیول 1](/docs/module-1-ros2) سے شروع کریں
4. AI chatbot سے سوالات پوچھیں

---

**نوٹ**: تمام تکنیکی اصطلاحات (<span className="technical-term">ROS 2</span>، <span className="technical-term">LLM</span>، <span className="technical-term">GPU</span>) درستگی کے لیے انگریزی میں محفوظ ہیں۔
