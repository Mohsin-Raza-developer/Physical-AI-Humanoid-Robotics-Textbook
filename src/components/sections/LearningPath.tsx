import React from 'react';
import { CheckCircleIcon } from '@heroicons/react/24/solid';
import styles from './LearningPath.module.css';

interface PathStepProps {
  number: string;
  title: string;
  description: string;
  topics: string[];
  isLast?: boolean;
}

function PathStep({ number, title, description, topics, isLast = false }: PathStepProps): JSX.Element {
  return (
    <div className={styles.pathStep}>
      <div className={styles.pathStepNumber}>{number}</div>
      {!isLast && <div className={styles.pathConnector} />}
      <div className={styles.pathStepContent}>
        <h3 className={styles.pathStepTitle}>{title}</h3>
        <p className={styles.pathStepDescription}>{description}</p>
        <ul className={styles.pathStepTopics}>
          {topics.map((topic, idx) => (
            <li key={idx} className={styles.pathStepTopic}>
              <CheckCircleIcon className={styles.pathStepTopicIcon} />
              <span>{topic}</span>
            </li>
          ))}
        </ul>
      </div>
    </div>
  );
}

export default function LearningPath(): JSX.Element {
  const learningSteps = [
    {
      number: '01',
      title: 'ROS 2 Fundamentals',
      description: 'Build a solid foundation in Robot Operating System 2',
      topics: ['Nodes & Topics', 'Services & Actions', 'Parameters & Launch Files'],
    },
    {
      number: '02',
      title: 'Simulation Mastery',
      description: 'Master simulation environments for robot testing',
      topics: ['Gazebo Integration', 'Unity Simulation', 'Physics Modeling'],
    },
    {
      number: '03',
      title: 'Isaac Sim Advanced',
      description: 'Leverage NVIDIA Isaac Sim for GPU-accelerated robotics',
      topics: ['Photorealistic Rendering', 'Sensor Simulation', 'Multi-Robot Systems'],
    },
    {
      number: '04',
      title: 'Vision-Language-Action',
      description: 'Implement AI models for embodied intelligence',
      topics: ['VLA Architecture', 'Multi-Modal Learning', 'Real-World Deployment'],
    },
  ];

  return (
    <section className={styles.learningPathSection}>
      <div className="container">
        <div className={styles.learningPathHeader}>
          <h2 className={styles.learningPathTitle}>Your Learning Journey</h2>
          <p className={styles.learningPathSubtitle}>
            Follow a structured path from basics to advanced robotics concepts
          </p>
        </div>
        <div className={styles.learningPathSteps}>
          {learningSteps.map((step, idx) => (
            <PathStep
              key={idx}
              number={step.number}
              title={step.title}
              description={step.description}
              topics={step.topics}
              isLast={idx === learningSteps.length - 1}
            />
          ))}
        </div>
      </div>
    </section>
  );
}
