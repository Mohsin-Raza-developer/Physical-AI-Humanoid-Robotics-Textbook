#!/usr/bin/env node

/**
 * Frontmatter Validation Script
 *
 * Validates that all Markdown files in the docs/ directory have complete frontmatter.
 * Required fields: title, sidebar_position, sidebar_label, description
 *
 * Usage: node scripts/validate-frontmatter.js
 */

const fs = require('fs');
const path = require('path');
const matter = require('gray-matter');

const DOCS_DIR = path.join(__dirname, '..', 'docs');
const REQUIRED_FIELDS = ['title', 'sidebar_position', 'sidebar_label'];
const RECOMMENDED_FIELDS = ['description', 'tags'];

let totalFiles = 0;
let validFiles = 0;
let filesWithWarnings = 0;
const errors = [];
const warnings = [];

/**
 * Recursively find all .md files in a directory
 */
function findMarkdownFiles(dir) {
  const files = [];
  const items = fs.readdirSync(dir);

  for (const item of items) {
    const fullPath = path.join(dir, item);
    const stat = fs.statSync(fullPath);

    if (stat.isDirectory()) {
      files.push(...findMarkdownFiles(fullPath));
    } else if (item.endsWith('.md') || item.endsWith('.mdx')) {
      files.push(fullPath);
    }
  }

  return files;
}

/**
 * Validate frontmatter for a single file
 */
function validateFile(filePath) {
  totalFiles++;

  const relativePath = path.relative(process.cwd(), filePath);
  const content = fs.readFileSync(filePath, 'utf-8');

  // Parse frontmatter
  let data;
  try {
    const parsed = matter(content);
    data = parsed.data;
  } catch (error) {
    errors.push(`${relativePath}: Failed to parse frontmatter - ${error.message}`);
    return;
  }

  // Check required fields
  const missingRequired = [];
  for (const field of REQUIRED_FIELDS) {
    if (!data[field]) {
      missingRequired.push(field);
    }
  }

  if (missingRequired.length > 0) {
    errors.push(`${relativePath}: Missing required fields: ${missingRequired.join(', ')}`);
    return;
  }

  // Check recommended fields
  const missingRecommended = [];
  for (const field of RECOMMENDED_FIELDS) {
    if (!data[field]) {
      missingRecommended.push(field);
    }
  }

  if (missingRecommended.length > 0) {
    warnings.push(`${relativePath}: Missing recommended fields: ${missingRecommended.join(', ')}`);
    filesWithWarnings++;
  }

  // Validation passed
  validFiles++;
}

/**
 * Main execution
 */
function main() {
  console.log('🔍 Validating frontmatter in docs/ directory...\n');

  if (!fs.existsSync(DOCS_DIR)) {
    console.error(`❌ Error: docs/ directory not found at ${DOCS_DIR}`);
    process.exit(1);
  }

  const markdownFiles = findMarkdownFiles(DOCS_DIR);

  if (markdownFiles.length === 0) {
    console.log('⚠️  No Markdown files found in docs/ directory');
    return;
  }

  // Validate each file
  for (const file of markdownFiles) {
    validateFile(file);
  }

  // Print results
  console.log('========================================');
  console.log(`📊 Validation Results`);
  console.log('========================================\n');
  console.log(`Total files checked: ${totalFiles}`);
  console.log(`✅ Valid files: ${validFiles}`);
  console.log(`⚠️  Files with warnings: ${filesWithWarnings}`);
  console.log(`❌ Files with errors: ${errors.length}\n`);

  if (errors.length > 0) {
    console.log('❌ ERRORS (Missing Required Fields):');
    console.log('========================================');
    errors.forEach(error => console.log(`  - ${error}`));
    console.log('');
  }

  if (warnings.length > 0) {
    console.log('⚠️  WARNINGS (Missing Recommended Fields):');
    console.log('========================================');
    warnings.forEach(warning => console.log(`  - ${warning}`));
    console.log('');
  }

  if (errors.length === 0 && warnings.length === 0) {
    console.log('✅ All files have complete frontmatter!\n');
  }

  // Exit with error code if there are errors
  if (errors.length > 0) {
    console.log('❌ Validation failed. Please fix the errors above.\n');
    process.exit(1);
  } else if (warnings.length > 0) {
    console.log('⚠️  Validation passed with warnings. Consider adding recommended fields.\n');
    process.exit(0);
  } else {
    console.log('✅ Validation passed successfully!\n');
    process.exit(0);
  }
}

// Run main function
if (require.main === module) {
  main();
}

module.exports = { validateFile, findMarkdownFiles };
