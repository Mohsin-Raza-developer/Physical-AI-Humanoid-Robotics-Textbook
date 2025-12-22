#!/usr/bin/env bash
# exit on error
set -o errexit

# Install dependencies from requirements.txt
pip install -r requirements.txt

# The custom openai-agents package is now in requirements.txt
# No need for a separate pip install command here.