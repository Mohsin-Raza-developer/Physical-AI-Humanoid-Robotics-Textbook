# Data Model: Docusaurus Book Setup

## Entities

### Module
- **Name**: String (required) - e.g., "ROS 2", "Gazebo/Unity", "Isaac", "VLA"
- **Description**: String (optional) - Brief description of the module
- **Slug**: String (required) - URL-friendly identifier
- **Order**: Integer (required) - Position in the learning sequence
- **Sections**: Array of Section entities - Content sections within the module
- **Metadata**: Object - Additional information like prerequisites, learning outcomes

### Section
- **Title**: String (required) - Section title
- **Slug**: String (required) - URL-friendly identifier
- **Order**: Integer (required) - Position within the module
- **Content**: String (required) - The actual content in Markdown/MDX format
- **Module**: Reference to Module entity
- **Prerequisites**: Array of string - Skills/knowledge needed before this section
- **LearningObjectives**: Array of string - What the student should learn
- **CodeExamples**: Array of CodeExample entities - Associated code samples

### CodeExample
- **Title**: String (optional) - Title for the code example
- **Language**: String (required) - One of: "python", "cpp", "urdf", "xml"
- **Code**: String (required) - The actual code content
- **Description**: String (optional) - Explanation of the code
- **Section**: Reference to Section entity

### ThemeConfiguration
- **Name**: String (required) - Theme identifier
- **IsDark**: Boolean (required) - Whether this is a dark theme
- **PrimaryColor**: String (optional) - Primary color in hex
- **SecondaryColor**: String (optional) - Secondary color in hex
- **BackgroundColor**: String (optional) - Background color in hex
- **TextColor**: String (optional) - Text color in hex
- **FontFamily**: String (optional) - Preferred font family

### UserPreference
- **UserId**: String (optional) - For future auth implementation
- **Theme**: String (required) - Current theme preference ("light" or "dark")
- **FontSize**: String (optional) - Preferred font size ("small", "medium", "large")
- **AccessibilitySettings**: Object - Additional accessibility preferences
- **LastVisited**: Timestamp (optional) - When the user last accessed content
- **Progress**: Array of objects - Tracking progress through modules/sections

## Relationships

```
Module --[1 to many]--> Section
Section --[1 to many]--> CodeExample
ThemeConfiguration --[1 to 1]--> UserPreference
Module --[1 to many]--> UserPreference (progress tracking)
```

## Validation Rules

1. **Module**:
   - Name must be unique across all modules
   - Order must be a positive integer
   - Sections must have unique slugs within the module

2. **Section**:
   - Title must not be empty
   - Order must be a positive integer
   - Content must be valid Markdown/MDX

3. **CodeExample**:
   - Language must be one of the supported languages: "python", "cpp", "urdf", "xml"
   - Code must be non-empty

4. **ThemeConfiguration**:
   - Name must be unique
   - Colors must be valid hex values or CSS color names

## State Transitions

### Module State
- **Draft**: Content is being created
- **Review**: Content is under review
- **Published**: Content is available to users
- **Archived**: Content is no longer active

### UserPreference State
- **New**: User hasn't set any preferences
- **Custom**: User has customized their preferences
- **Default**: Using default platform settings