import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Physical AI & Humanoid Robotics Book Sidebar
 *
 * Manual sidebar configuration for the Physical AI course content.
 * Includes Introduction, Hardware Requirements, and 4 Course Modules.
 */
const sidebars: SidebarsConfig = {
  physicalAISidebar: [
    'introduction',
    'hardware-requirements',
    {
      type: 'category',
      label: 'Course Modules',
      items: [
        'module-1-ros2',
        'module-2-gazebo-unity',
        'module-3-nvidia-isaac',
        'module-4-vla',
      ],
    },
  ],
};

export default sidebars;
