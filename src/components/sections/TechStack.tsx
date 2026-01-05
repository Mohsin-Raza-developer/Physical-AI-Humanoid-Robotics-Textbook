import React from 'react';
import styles from './TechStack.module.css';

interface TechItemProps {
  name: string;
  description: string;
  category: string;
}

function TechItem({ name, description, category }: TechItemProps): JSX.Element {
  return (
    <div className={styles.techItem}>
      <div className={styles.techItemHeader}>
        <div className={styles.techItemCategory}>{category}</div>
      </div>
      <h3 className={styles.techItemName}>{name}</h3>
      <p className={styles.techItemDescription}>{description}</p>
    </div>
  );
}

export default function TechStack(): JSX.Element {
  const technologies = [
    {
      name: 'ROS 2',
      category: 'Framework',
      description: 'Modern robot operating system for distributed robotics applications',
    },
    {
      name: 'Gazebo',
      category: 'Simulation',
      description: 'Open-source 3D robotics simulator with accurate physics',
    },
    {
      name: 'Unity',
      category: 'Simulation',
      description: 'Game engine for creating realistic robot simulation environments',
    },
    {
      name: 'Isaac Sim',
      category: 'Platform',
      description: 'NVIDIA\'s GPU-accelerated robotics simulation platform',
    },
    {
      name: 'Python',
      category: 'Language',
      description: 'Primary programming language for robotics and AI development',
    },
    {
      name: 'C++',
      category: 'Language',
      description: 'High-performance language for real-time robotics systems',
    },
    {
      name: 'Vision-Language Models',
      category: 'AI/ML',
      description: 'Multi-modal models for embodied AI and robot decision-making',
    },
    {
      name: 'Action Models',
      category: 'AI/ML',
      description: 'Learning-based control systems for robotic manipulation',
    },
  ];

  return (
    <section className={styles.techStackSection}>
      <div className="container">
        <div className={styles.techStackHeader}>
          <h2 className={styles.techStackTitle}>Technology Stack</h2>
          <p className={styles.techStackSubtitle}>
            Learn industry-standard tools and frameworks used in modern robotics
          </p>
        </div>
        <div className={styles.techStackGrid}>
          {technologies.map((tech, idx) => (
            <TechItem
              key={idx}
              name={tech.name}
              description={tech.description}
              category={tech.category}
            />
          ))}
        </div>
      </div>
    </section>
  );
}
