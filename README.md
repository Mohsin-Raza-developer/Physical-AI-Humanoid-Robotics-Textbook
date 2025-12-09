# Physical AI & Humanoid Robotics Textbook

Interactive textbook for Physical AI and Humanoid Robotics education built with Docusaurus v3.

## Features

- 4 comprehensive modules covering ROS 2, Gazebo/Unity, Isaac, and Vision-Language-Action (VLA) models
- Premium UI/UX with responsive design for all device sizes
- Syntax highlighting for Python, C++, URDF, and XML code examples
- Dark/light mode toggle with system preference detection
- WCAG 2.1 AA accessibility compliance
- GitHub Pages deployment with automated CI/CD

## Tech Stack

- [Docusaurus v3](https://docusaurus.io/) - Static site generator optimized for documentation
- [React](https://reactjs.org/) - Component-based UI library
- [Tailwind CSS](https://tailwindcss.com/) - Utility-first CSS framework
- [TypeScript](https://www.typescriptlang.org/) - Typed superset of JavaScript
- [Node.js](https://nodejs.org/) - JavaScript runtime environment
- [GitHub Actions](https://github.com/features/actions) - Automated deployment

## Local Development

### Prerequisites

- [Node.js](https://nodejs.org/en/download/) version 20.0 or above
- [npm](https://www.npmjs.com/) package manager (comes with Node.js)

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/Mohsin-Raza-developer/Physical-AI-Humanoid-Robotics-Textbook.git
   cd Physical-AI-Humanoid-Robotics-Textbook
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   npm start
   ```

This command starts a local development server and opens the application in your browser. Most changes are reflected live without restarting the server.

### Project Structure

```
Physical-AI-Humanoid-Robotics-Textbook/
├── docs/                 # Documentation content
│   ├── ros2/             # ROS 2 module content
│   ├── gazebo-unity/     # Gazebo/Unity module content
│   ├── isaac/            # Isaac framework module content
│   ├── vla/              # Vision-Language-Action module content
│   └── intro/            # Introduction content
├── src/                  # Custom React components and styles
│   ├── components/       # Reusable React components
│   ├── pages/            # Custom pages
│   ├── css/              # Custom styles
│   └── theme/            # Custom theme components
├── static/               # Static assets
├── .github/              # GitHub Actions workflows
├── docusaurus.config.ts  # Docusaurus configuration
├── sidebars.ts           # Navigation sidebar configuration
└── package.json          # Dependencies and scripts
```

## Contributing

### Adding Content

1. Create a new markdown file in the appropriate module directory (e.g., `docs/ros2/new-topic.md`)
2. Include frontmatter metadata:
   ```markdown
   ---
   title: New Topic
   sidebar_position: 1
   description: Description of the new topic
   ---
   
   # New Topic
   
   Content goes here...
   ```
3. Add your content using Markdown syntax
4. Update the sidebar by modifying `sidebars.ts` if needed

### Adding Code Examples

Code blocks with appropriate language tags will be automatically syntax-highlighted:

```python
# Python example
import rospy
from geometry_msgs.msg import Twist

def move_robot():
    # Implementation here
    pass
```

Available syntax highlighting languages include:
- `python` - Python code examples
- `cpp` - C++ code examples
- `xml` - XML and URDF files
- `bash` - Shell commands
- `json` - JSON examples
- `yaml` - YAML examples

### Custom Components

To add a new custom component:
1. Create the component in `src/components/`
2. Export it properly
3. Import and use in your documentation pages

## Building for Production

To build the website for production:

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static content hosting service.

## Deployment

The site is configured for deployment to GitHub Pages using GitHub Actions. The workflow is defined in `.github/workflows/deploy.yml`.

Changes pushed to the `main` branch will trigger an automatic deployment to the GitHub Pages site.

## Accessibility

This website follows WCAG 2.1 AA accessibility guidelines:
- Sufficient color contrast in both light and dark modes
- Semantic HTML structure
- Keyboard navigation support
- Responsive design for various screen sizes
- Screen reader compatibility

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

For questions or issues, please open an issue on the GitHub repository.