import cohere
from typing import List, Dict, Any
from pydantic import BaseModel
from dotenv import load_dotenv
import os

load_dotenv()

class CohereConfig:
    api_key = os.getenv("COHERE_API_KEY")
    model = "command-r-plus"  # Using Cohere's most capable model

class CohereService:
    def __init__(self):
        if not CohereConfig.api_key:
            raise ValueError("COHERE_API_KEY environment variable is not set")
        
        self.client = cohere.Client(CohereConfig.api_key)
        self.model = CohereConfig.model

    def generate_response(self, prompt: str, context: str = "") -> str:
        """
        Generate a response using Cohere's generate endpoint
        """
        if context:
            full_prompt = f"Context: {context}\n\nQuestion: {prompt}\n\nPlease answer based only on the provided context. If the context does not contain enough information to answer the question, respond with: 'The selected text does not contain enough information to answer this question.'"
        else:
            full_prompt = f"Question: {prompt}\n\nPlease answer based only on the provided context. If the context does not contain enough information to answer the question, respond with: 'The selected text does not contain enough information to answer this question.'"

        response = self.client.generate(
            model=self.model,
            prompt=full_prompt,
            max_tokens=500,
            temperature=0.3  # Lower temperature for more consistent, factual responses
        )
        
        return response.generations[0].text.strip()

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embeddings for a given text
        """
        response = self.client.embed(
            texts=[text],
            model="embed-english-v3.0",
            input_type="search_document"
        )
        return response.embeddings[0]

    def embed_query(self, query: str) -> List[float]:
        """
        Generate embeddings for a query
        """
        response = self.client.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        return response.embeddings[0]