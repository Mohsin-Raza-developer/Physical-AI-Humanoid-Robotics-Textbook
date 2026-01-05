import type {ReactNode} from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Hero from '@site/src/components/sections/Hero';
import StatsCounter from '@site/src/components/sections/StatsCounter';
import LearningPath from '@site/src/components/sections/LearningPath';
import TechStack from '@site/src/components/sections/TechStack';
import CTASection from '@site/src/components/sections/CTASection';

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome`}
      description="Physical AI & Humanoid Robotics Interactive Textbook - Learn ROS 2, Gazebo, Isaac Sim, and VLA models">
      <Hero />
      <main>
        <StatsCounter />
        <HomepageFeatures />
        <LearningPath />
        <TechStack />
        <CTASection />
      </main>
    </Layout>
  );
}
