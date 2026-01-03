"""
Hugging Face Space Entry Point for FastAPI Backend

This file serves as the entry point for Hugging Face Spaces deployment.
It imports and exposes the FastAPI app from src/main.py
"""

from src.main import app

# Hugging Face Spaces will automatically detect and run this app
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=7860)
