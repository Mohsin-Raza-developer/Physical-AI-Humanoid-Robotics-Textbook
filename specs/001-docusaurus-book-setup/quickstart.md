# Quickstart Guide: Docusaurus Book Setup

## Prerequisites

- Node.js 18.0 or higher
- npm (comes with Node.js) or yarn
- Git

## Installation

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install dependencies**
   ```bash
   npm install
   # or
   yarn install
   ```

3. **Start the development server**
   ```bash
   npm start
   # or
   yarn start
   ```
   
   This command starts a local development server and opens the application in your browser. Most changes are reflected live without restarting the server.

## Project Structure

```
docs/
├── docs/
│   ├── ros2/            # ROS 2 module content
│   ├── gazebo-unity/    # Gazebo/Unity module content
│   ├── isaac/           # Isaac module content
│   ├── vla/             # VLA module content
│   └── intro/           # Introduction content
├── src/
│   ├── components/      # Custom React components
│   ├── pages/           # Custom pages
│   ├── css/             # Custom styles (Tailwind + custom)
│   └── theme/           # Custom theme components
├── static/              # Static assets (images, files)
├── docusaurus.config.js # Docusaurus configuration
├── sidebars.js          # Navigation configuration
├── package.json         # Dependencies and scripts
├── tailwind.config.js   # Tailwind CSS configuration
└── babel.config.js      # Babel configuration
```

## Adding Content

1. **Create a new document**
   - Add a Markdown file in the appropriate module directory (e.g., `docs/docs/ros2/new-topic.md`)
   - Include frontmatter metadata:
   
   ```markdown
   ---
   title: New Topic
   sidebar_position: 1
   description: Description of the new topic
   ---
   
   # New Topic
   
   Content goes here...
   ```

2. **Update the sidebar**
   - Edit `sidebars.js` to add your new document to the navigation

3. **Add code examples**
   - Code blocks with appropriate language tags will be automatically syntax-highlighted
   - Example: ` ```python ` for Python code

## Custom Styling

1. **Tailwind CSS**
   - Configuration is in `tailwind.config.js`
   - Custom styles can be added in `src/css/custom.css`

2. **Custom Components**
   - Add React components in `src/components/`
   - Use them in MDX files with standard React import syntax

## Building for Production

```bash
npm run build
# or
yarn build
```

This command generates static content into the `build` directory and can be served using any static content hosting service.

## Deployment

The project is configured for deployment to GitHub Pages using GitHub Actions. The workflow is defined in `.github/workflows/deploy.yml`.

## Testing

1. **Local Development**
   - Changes are reflected live in the development server

2. **Production Build Test**
   ```bash
   npm run serve
   # or
   yarn serve
   ```
   
   This command builds the project and serves the production build locally for testing.

## Accessibility Features

- WCAG 2.1 AA compliant by default through Docusaurus
- Dark mode toggle available in the header
- Responsive design for mobile, tablet, and desktop
- Proper semantic HTML structure
- ARIA attributes where appropriate