import type { ReactNode } from "react";
import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import Heading from "@theme/Heading";

import styles from "./index.module.css";
import moduleStyles from "./index.module.css"; // ⭐ NEW CSS

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx("hero hero--primary", styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>

        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="http://localhost:3000/docs/"
          >
            View Textbook 📘
          </Link>
        </div>
      </div>
    </header>
  );
}

/* ⭐ NEW MODULE SECTION */
function ModulesSection() {
  return (
    <section className={moduleStyles.wrapper}>
      <h2 className={moduleStyles.title}>Start Learning — 4 Core Modules</h2>

      <div className={moduleStyles.moduleGrid}>
        <Link
          className={moduleStyles.card}
          to="/docs/content/modules/ros2-fundamentals/intro"
        >
          <h3>Module 1: ROS 2 Fundamentals</h3>
          <p>Learn ROS 2 basics, nodes, topics, and communication.</p>
        </Link>

        <Link
          className={moduleStyles.card}
          to="/docs/content/modules/digital-twin/intro"
        >
          <h3>Module 2: Digital Twin</h3>
          <p>Build simulations using Gazebo & Unity environments.</p>
        </Link>

        <Link
          className={moduleStyles.card}
          to="/docs/content/modules/ai-robot-brain/intro"
        >
          <h3>Module 3: AI-Robot Brain</h3>
          <p>AI pipelines, GPU robotic learning and control.</p>
        </Link>

        <Link
          className={moduleStyles.card}
          to="/docs/content/modules/vla/intro"
        >
          <h3>Module 4: Vision-Language-Action</h3>
          <p>VLA models, robotic perception and action planning.</p>
        </Link>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Welcome | ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Textbook"
    >
      <HomepageHeader />

      <main>
        {/* ⭐ Modules Section Added Here */}
        <ModulesSection />
      </main>
    </Layout>
  );
}
