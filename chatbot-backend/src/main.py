"""
FastAPI Main Application

Entry point for the chatbot backend API with CORS configuration
and health check endpoint.

Maps to: FR-015
"""

import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Create FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook Chatbot API",
    description="RAG-based chatbot backend for interactive textbook",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# Configure CORS
CORS_ORIGINS = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")

app.add_middleware(
    CORSMiddleware,
    allow_origins=CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/health")
async def health_check():
    """
    Health check endpoint

    Returns:
        200 OK with status message
    """
    return JSONResponse(
        status_code=200,
        content={
            "status": "ok",
            "message": "Chatbot backend is running"
        }
    )


@app.get("/")
async def root():
    """
    Root endpoint

    Returns:
        API information
    """
    return {
        "name": "Physical AI & Humanoid Robotics Textbook Chatbot API",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/health"
    }


# Startup event
@app.on_event("startup")
async def startup_event():
    """Initialize services on startup"""
    print("🚀 Starting Chatbot Backend API...")
    print(f"   Environment: {os.getenv('ENVIRONMENT', 'development')}")
    print(f"   CORS origins: {CORS_ORIGINS}")
    print("✅ Backend ready!")


# Shutdown event
@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown"""
    print("👋 Shutting down Chatbot Backend API...")
