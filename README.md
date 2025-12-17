# Physical AI & Humanoid Robotics — AI-Native Textbook

This project contains an AI-native textbook for Physical AI & Humanoid Robotics, featuring interactive learning tools and comprehensive coverage of modern robotics technologies.

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Getting Started](#getting-started)
4. [Project Structure](#project-structure)
5. [Technology Stack](#technology-stack)
6. [Contributing](#contributing)
7. [License](#license)

## Overview

This AI-native textbook provides comprehensive coverage of Physical AI & Humanoid Robotics, including:

- ROS 2 fundamentals for humanoid robotics
- Digital twin technology with Gazebo and Unity
- AI-Robot brains using NVIDIA Isaac
- Vision-Language-Action robotics
- Capstone project integrating all concepts

## Features

- **Interactive Learning**: Embedded RAG chatbot that answers questions based on the textbook content
- **Modular Design**: Organized into distinct modules for focused learning
- **Accessibility**: Includes dark/light mode, high contrast support, and keyboard navigation
- **Validation Checklists**: Each module includes validation checklists to verify understanding
- **AI Integration**: RAG system that answers questions without hallucination

## Getting Started

### Frontend (Docusaurus Book)

1. Install dependencies:
   ```bash
   npm install
   ```

2. Start the development server:
   ```bash
   npm start
   ```

3. Open [http://localhost:3000](http://localhost:3000) to view the textbook in your browser.

### Backend (RAG Chatbot)

1. Create a virtual environment and install dependencies:
   ```bash
   python -m venv .venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   pip install -r backend/requirements.txt
   ```

2. Set up environment variables:
   ```bash
   cp backend/.env.example backend/.env
   # Edit backend/.env with your configuration
   ```

3. Start the backend server:
   ```bash
   cd backend
   python -m src.api.main
   ```

## Project Structure

```
.
├── backend/                 # FastAPI backend for RAG chatbot
│   ├── src/
│   │   ├── api/            # API endpoints
│   │   ├── models/         # Data models
│   │   ├── services/       # Business logic
│   │   ├── config/         # Configuration management
│   │   └── utils/          # Utility functions
│   └── requirements.txt
├── docs/                  # Docusaurus documentation (the textbook)
│   ├── docs/              # Textbook content
│   ├── src/               # Custom components
│   ├── static/            # Static assets
│   ├── docusaurus.config.js
│   └── sidebars.js
├── src/                   # Additional frontend components
├── .github/               # GitHub workflows
├── .specify/              # SpecKit Plus configuration
└── README.md
```

## Technology Stack

### Frontend
- [Docusaurus](https://docusaurus.io/): Static site generator optimized for documentation
- [React](https://reactjs.org/): JavaScript library for building user interfaces
- [MDX](https://mdxjs.com/): JSX in Markdown files for interactive content
- [Tailwind CSS](https://tailwindcss.com/): Utility-first CSS framework

### Backend
- [Python](https://www.python.org/): Programming language
- [FastAPI](https://fastapi.tiangolo.com/): Modern, fast web framework
- [Qdrant](https://qdrant.tech/): Vector database for semantic search
- [PostgreSQL](https://www.postgresql.org/): Relational database
- [Pydantic](https://pydantic-docs.helpmanual.io/): Data validation library

### Deployment
- GitHub Pages: Static hosting for the textbook
- Containerization: For backend deployment

## Contributing

We welcome contributions from the community! Here's how you can contribute:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

Please make sure to update tests as appropriate and follow the code style guidelines.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Special thanks to the open-source communities that made this project possible
- The Docusaurus team for providing an excellent documentation platform
- The FastAPI community for the amazing web framework