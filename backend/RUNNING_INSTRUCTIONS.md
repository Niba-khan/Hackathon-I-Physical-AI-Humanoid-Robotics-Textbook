# Running Backend and Frontend Together

## Prerequisites
- Python 3.11+
- Node.js (if using a separate frontend framework)
- Access to required API keys (Cohere, Qdrant, Neon)

## Running the Backend

### 1. Setup Backend Environment
```bash
# Navigate to the backend directory
cd C:\Users\HT\Desktop\hackathon2\backend

# Create and activate virtual environment
python -m venv venv
venv\Scripts\activate  # On Windows

# Install dependencies
pip install -r requirements.txt
```

### 2. Set Environment Variables
Create a `.env` file in the backend directory with the following variables:
```env
COHERE_API_KEY=your_cohere_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_url
NEON_DATABASE_URL=your_neon_database_url
SECRET_KEY=your_secret_key_for_session_management
```

### 3. Run Database Migrations
```bash
# Run database migrations
alembic upgrade head
```

### 4. Index Book Content (One-time Setup)
```bash
# Run the content ingestion pipeline
python -m src.services.ingestion_pipeline
```

### 5. Start Backend Server
```bash
# Start the backend server
uvicorn src.main:app --host 0.0.0.0 --port 8000
```

The backend will now be running at `http://localhost:8000`

## Running the Frontend

### Option 1: Using the Sample HTML File
1. Open the `frontend_integration_example.html` file in your browser
2. The sample frontend is already configured to connect to `http://localhost:8000`
3. You can start asking questions to test the integration

### Option 2: If Using Docusaurus (Typical for Textbook)
If your textbook is built with Docusaurus, follow these steps:

```bash
# Navigate to the textbook directory (separate from backend)
cd path/to/your/textbook

# Install dependencies
npm install

# Start the Docusaurus development server
npm run start
```

The Docusaurus server typically runs at `http://localhost:3000`.

## Important Configuration for Development

### Backend CORS Settings
If your frontend runs on a different port than the backend (common in development), you may need to configure CORS in the backend. Add the following to your `src/main.py`:

```python
from fastapi.middleware.cors import CORSMiddleware

# Add this after creating the FastAPI app
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Add your frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### Environment-Specific Configuration
For development, you might want to add CORS support to your backend. Create a new file `src/core/middleware.py`:

```python
from fastapi.middleware.cors import CORSMiddleware
from src.core.config import settings

def setup_cors(app):
    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.allowed_origins or ["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
```

Then in `src/main.py`, call this function after creating the app:
```python
from src.core.middleware import setup_cors

# After creating app instance
setup_cors(app)
```

And add to your `src/core/config.py`:
```python
allowed_origins: list = ["http://localhost:3000"]  # Add your frontend URLs
```

## Testing the Integration

1. Start the backend server as described above
2. Start your frontend (either the sample HTML or Docusaurus)
3. In the frontend, try asking a question about the textbook content
4. Verify that:
   - The request is sent to the backend
   - The backend processes the request and returns a response
   - The response is displayed in the frontend
   - Sources are properly shown if available

## Troubleshooting Common Issues

### CORS Errors
If you see CORS errors in the browser console, ensure the backend allows requests from your frontend origin.

### API Connection Issues
- Verify the backend is running and accessible at the configured URL
- Check that the API endpoints are correctly formatted
- Confirm that environment variables are properly set

### Mode-Specific Issues
- For "selected-text-only" mode, ensure text is provided in the selected_text field
- For "book-wide" mode, ensure the vector store is properly indexed with content

## Production Deployment Considerations

When deploying to production:
1. Update the backend URL in the frontend to point to your production backend
2. Configure proper CORS settings for your production domains
3. Ensure all API keys are properly secured
4. Set up proper SSL certificates for secure communication
5. Implement proper error handling and logging