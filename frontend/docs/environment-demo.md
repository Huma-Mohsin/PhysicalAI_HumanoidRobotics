---
sidebar_position: 7
title: Environment Mode Demo
---

import { EnvironmentSwitcher } from '@site/src/components/EnvironmentSwitcher';
import { WorkstationOnly, CloudOnly, MacOnly, CloudOrMac, EnvironmentContent } from '@site/src/components/EnvironmentContent';

# Environment Mode Demo

This page demonstrates the environment-aware content filtering system.

<EnvironmentSwitcher />

## How It Works

The platform adapts content based on your hardware profile:

- **💻 Workstation Mode**: Shows local Isaac Sim installation guides (requires RTX 4070 Ti+)
- **☁️ Cloud Mode**: Shows Omniverse Cloud workflows
- **🍎 Mac Mode**: Shows macOS-compatible alternatives

Switch modes above to see content change dynamically!

---

## Example: Environment-Specific Installation

<WorkstationOnly>

### 💻 Workstation: Local Isaac Sim Setup

You have a compatible GPU! Follow these steps for local installation:

```bash
# Download Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage

# Install Isaac Sim 2023.1
# Via Launcher → Exchange → Isaac Sim → Install
```

**Your GPU**: {user?.hardware_profile?.gpu_model || 'RTX 4070 Ti+'}

**Benefits**:
- ✅ Unlimited simulation time
- ✅ Full RTX ray tracing
- ✅ Offline workflow
- ✅ Custom USD scene editing

</WorkstationOnly>

<CloudOnly>

### ☁️ Cloud: Omniverse Cloud Access

Perfect for users without local GPU! Access Isaac Sim via browser:

1. Visit [Omniverse Cloud](https://www.nvidia.com/en-us/omniverse/cloud/)
2. Sign in with NVIDIA account
3. Launch Isaac Sim from dashboard
4. Stream directly to your browser

**Benefits**:
- ✅ No GPU required
- ✅ Access from any device
- ✅ Pre-configured environments
- ✅ Automatic updates

**Note**: Free tier limited to 2-hour sessions

</CloudOnly>

<MacOnly>

### 🍎 Mac: Alternative Workflows

Isaac Sim doesn't run natively on macOS. Here are your options:

**Option 1: Omniverse Cloud** (Recommended)
- Stream Isaac Sim via browser
- See Cloud Mode instructions

**Option 2: PyBullet Simulation**
```bash
pip install pybullet
```

**Option 3: Gazebo via Homebrew**
```bash
brew install gazebo
```

**Option 4: Docker + Cloud GPU**
- Use Docker to run Isaac Sim remotely
- Connect to cloud GPU instance

</MacOnly>

---

## Example: Hardware-Specific Workflows

<EnvironmentContent modes={['workstation', 'cloud']}>

### Training Large Models

Both Workstation and Cloud users can train models, but with different approaches:

**Workstation**: Train locally with full GPU access
**Cloud**: Generate data in cloud, train on local machine or Colab

</EnvironmentContent>

<MacOnly>

### Mac-Specific Considerations

- Use MPS backend for Apple Silicon acceleration
- Consider remote GPU via Paperspace/Colab
- Focus on lightweight simulation alternatives

</MacOnly>

---

## Example: Conditional Code Blocks

<WorkstationOnly>

```python
# Workstation: Full Isaac Sim with RTX
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting",  # RTX ray tracing
    "width": 1920,
    "height": 1080
})
```

</WorkstationOnly>

<CloudOnly>

```python
# Cloud: Optimized settings for streaming
isaac_cloud.launch_session(
    quality="medium",  # Balance performance and bandwidth
    session_timeout=7200  # 2 hours max
)
```

</CloudOnly>

<MacOnly>

```python
# Mac: PyBullet alternative
import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
robot = p.loadURDF("r2d2.urdf")
```

</MacOnly>

---

## Testing Your Environment

<EnvironmentContent modes={['workstation']}>

### ✅ Workstation Mode Active

You're seeing workstation-specific content. This means:
- You have a compatible GPU (RTX 4070 Ti+)
- Local Isaac Sim installation guides are shown
- Full performance examples included

</EnvironmentContent>

<EnvironmentContent modes={['cloud']}>

### ✅ Cloud Mode Active

You're seeing cloud-optimized content. This means:
- Browser-based workflows prioritized
- Session time considerations included
- Hybrid cloud/local workflows suggested

</EnvironmentContent>

<EnvironmentContent modes={['mac']}>

### ✅ Mac Mode Active

You're seeing Mac-compatible content. This means:
- Alternative simulation tools highlighted
- MPS acceleration tips included
- Remote GPU options suggested

</EnvironmentContent>

---

## Updating Your Environment Preference

To change your hardware profile:

1. Go to [Profile Settings](/profile)
2. Update your GPU model
3. Select preferred environment mode
4. Save changes

The platform will automatically adjust content across all modules!

---

## Technical Implementation

This feature uses:
- **React Context** for global environment state
- **Conditional Rendering** based on user's hardware profile
- **MDX Components** for inline environment filtering
- **LocalStorage** for manual mode overrides

**Source Code**: `frontend/src/contexts/EnvironmentContext.tsx`
