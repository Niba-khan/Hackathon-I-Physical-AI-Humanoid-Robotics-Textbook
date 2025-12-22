# Quickstart Guide: Integrated RAG Chatbot for AI-Native Textbook

## Prerequisites
- Python 3.11+
- pip package manager
- Git
- Access to Cohere API (with valid API key)
- Access to Qdrant Cloud (with valid URL and API key)
- Access to Neon Serverless Postgres (with connection URL)

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd hackathon2/backend
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

### 4. Set Environment Variables
Create a `.env` file in the backend directory with the following variables:

```env
COHERE_API_KEY=your_cohere_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_url
NEON_DATABASE_URL=your_neon_database_url
SECRET_KEY=your_secret_key_for_session_management
```

### 5. Run Database Migrations
```bash
# This would run your database migration tool
python -m src.core.database migrate
```

### 6. Index Book Content (One-time Setup)
```bash
# This would run the content ingestion pipeline
python -m src.services.ingestion_pipeline
```

### 7. Start the Backend Server
```bash
uvicorn src.main:app --host 0.0.0.0 --port 8000
```

## API Endpoints

### Chat Endpoint
- **POST** `/api/chat`
- Request body:
```json
{
  "query": "Your question here",
  "mode": "book-wide" | "selected-text-only",
  "selected_text": "Text selected by user (required for selected-text-only mode)"
}
```
- Response:
```json
{
  "response": "The chatbot's answer",
  "sources": ["list of source references"],
  "session_id": "unique session identifier"
}
```

### Health Check
- **GET** `/health`
- Returns service health status

## Frontend Integration

The chatbot widget should be embedded in the Docusaurus book UI. The frontend needs to:
1. Capture user queries and selected text
2. Call the backend API with appropriate mode
3. Display responses with source citations
4. Manage session continuity

## Testing

### Run Unit Tests
```bash
pytest tests/unit/
```

### Run Integration Tests
```bash
pytest tests/integration/
```

### Run Contract Tests
```bash
pytest tests/contract/
```

## Troubleshooting

### Common Issues
- **API Key Errors**: Verify all environment variables are set correctly
- **Vector Store Connection**: Check QDRANT_URL and QDRANT_API_KEY
- **Database Connection**: Verify NEON_DATABASE_URL format
- **Cohere API Errors**: Check COHERE_API_KEY and rate limits