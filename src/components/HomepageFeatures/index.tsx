import type {ReactNode} from 'react';
import { CpuChipIcon, RocketLaunchIcon, BoltIcon, AcademicCapIcon } from '@heroicons/react/24/outline';
import FeatureCard from '../features/FeatureCard';
import styles from './styles.module.css';

type FeatureItem = {
  icon: ReactNode;
  title: string;
  description: string;
  link: string;
};

const FeatureList: FeatureItem[] = [
  {
    icon: <CpuChipIcon className="w-14 h-14" />,
    title: 'ROS 2 Fundamentals',
    description: 'Master Robot Operating System 2 with hands-on tutorials covering nodes, topics, services, and actions for modern robotics development.',
    link: '/docs/ros2/intro',
  },
  {
    icon: <RocketLaunchIcon className="w-14 h-14" />,
    title: 'Simulation Environments',
    description: 'Learn Gazebo and Unity simulation tools to test and validate robot behaviors in realistic virtual environments before deployment.',
    link: '/docs/gazebo-unity/intro',
  },
  {
    icon: <BoltIcon className="w-14 h-14" />,
    title: 'Isaac Sim Platform',
    description: 'Explore NVIDIA Isaac Sim for GPU-accelerated robotics simulation with photorealistic rendering and physics-based modeling.',
    link: '/docs/isaac/intro',
  },
  {
    icon: <AcademicCapIcon className="w-14 h-14" />,
    title: 'Vision-Language-Action',
    description: 'Understand VLA models that combine computer vision and language understanding to enable embodied AI decision-making.',
    link: '/docs/vla/intro',
  },
];

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.featuresGrid}>
          {FeatureList.map((feature, idx) => (
            <FeatureCard
              key={idx}
              icon={feature.icon}
              title={feature.title}
              description={feature.description}
              link={feature.link}
            />
          ))}
        </div>
      </div>
    </section>
  );
}
