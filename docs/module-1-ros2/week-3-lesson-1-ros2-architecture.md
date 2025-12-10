---
title: ROS 2 Architecture
sidebar_label: "Lesson 1: Architecture"
sidebar_position: 31
description: Understanding the architecture of Robot Operating System 2
tags: [ros2, week-3, architecture]
---

# ROS 2 Architecture

The Robot Operating System 2 (ROS 2) is designed to be a flexible framework for writing robot software. Unlike traditional approaches, ROS 2 is not an operating system but a collection of tools and libraries that provide implementation for many of the functions that a robot needs.

## Key Concepts

:::info Prerequisites
Before diving into ROS 2 architecture, ensure you have a basic understanding of distributed systems and inter-process communication. Familiarity with Linux and command-line interfaces will be helpful.
:::

- Client Library Implementations
- DDS/RMW Abstraction
- Packages and Composable Nodes
- Parameters and Configuration

## DDS and RMW

:::tip Why DDS?
ROS 2 chose DDS (Data Distribution Service) as its middleware because it provides real-time, reliable communication and is an industry standard (OMG). This makes ROS 2 suitable for production robotics systems, unlike ROS 1.
:::

The Data Distribution Service (DDS) and the ROS Middleware (RMW) abstraction layer are fundamental to ROS 2's design.