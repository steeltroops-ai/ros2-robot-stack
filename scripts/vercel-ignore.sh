#!/bin/bash

# Vercel Ignored Build Step Script
# This prevents Vercel from double-deploying when we use GitHub Actions.

echo "--- [VERCEL IGNORE] Checking build trigger... ---"

# If the build is triggered by the GitHub App integration (automatic push), 
# we want to CANCEL it (exit 0) because our GitHub Action will trigger it instead.
if [[ "$VERCEL_GIT_COMMIT_REF" == "main" && "$VERCEL_ENV" == "production" ]]; then
  echo "🛑 Cancelling automatic build for 'main'. GitHub Action will handle this."
  exit 0
else
  # Proceed with the build for preview branches or if triggered by our Action
  echo "✅ Proceeding with build."
  exit 1
fi
