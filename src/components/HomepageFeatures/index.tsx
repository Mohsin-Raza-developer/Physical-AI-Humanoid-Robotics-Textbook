import type {ReactNode} from 'react';
import Icon from '../ui/Icon';
import FeatureCard from '../features/FeatureCard';
import AnimatedSection from '../ui/AnimatedSection';
import styles from './styles.module.css';

type FeatureItem = {
  icon: ReactNode;
  title: string;
  description: string;
  link: string;
};

const FeatureList: FeatureItem[] = [
  {
    icon: <Icon name="cpu" size="xl" ariaLabel="ROS 2 icon" />,
    title: 'ROS 2 Fundamentals',
    description: 'Master Robot Operating System 2 with hands-on tutorials covering nodes, topics, services, and actions for modern robotics development.',
    link: '/docs/module-1-ros2/intro',
  },
  {
    icon: <Icon name="rocket" size="xl" ariaLabel="Simulation icon" />,
    title: 'Simulation Environments',
    description: 'Learn Gazebo and Unity simulation tools to test and validate robot behaviors in realistic virtual environments before deployment.',
    link: '/docs/module-2-gazebo-unity/intro',
  },
  {
    icon: <Icon name="bolt" size="xl" ariaLabel="Isaac Sim icon" />,
    title: 'Isaac Sim Platform',
    description: 'Explore NVIDIA Isaac Sim for GPU-accelerated robotics simulation with photorealistic rendering and physics-based modeling.',
    link: '/docs/module-3-isaac/intro',
  },
  {
    icon: <Icon name="academic" size="xl" ariaLabel="Vision-Language-Action icon" />,
    title: 'Vision-Language-Action',
    description: 'Understand VLA models that combine computer vision and language understanding to enable embodied AI decision-making.',
    link: '/docs/module-4-vla/intro',
  },
];

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.featuresGrid}>
          {FeatureList.map((feature, idx) => (
            <AnimatedSection
              key={idx}
              animation="fade-in"
              delay={idx * 100}
              threshold={0.1}
            >
              <FeatureCard
                icon={feature.icon}
                title={feature.title}
                description={feature.description}
                link={feature.link}
              />
            </AnimatedSection>
          ))}
        </div>
      </div>
    </section>
  );
}
