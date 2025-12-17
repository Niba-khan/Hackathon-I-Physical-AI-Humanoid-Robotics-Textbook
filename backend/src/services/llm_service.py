import requests
import json
from typing import List, Dict, Any
from config import settings
from utils import logger


class LLMService:
    def __init__(self):
        # Using a non-OpenAI LLM provider via API (placeholder implementation)
        # In a real implementation, this would connect to a non-OpenAI LLM provider
        self.api_key = settings.llm_provider_api_key
        self.api_url = "https://api.llm-provider.example.com/v1/chat/completions"  # Placeholder
    
    def generate_response(self, question: str, context: str) -> str:
        """
        Generate a response using the LLM based on the question and context.
        
        Args:
            question: The user's question
            context: The context to ground the answer in
        Returns:
            Generated response
        """
        # Create the grounded prompt
        prompt = self._create_grounding_prompt(question, context)
        
        # Prepare the request payload
        payload = {
            "model": "non-openai-model",  # Using a placeholder model name
            "messages": [
                {
                    "role": "system",
                    "content": "You are a helpful assistant that answers questions based only on the provided context. If the answer is not in the context, respond with 'This information is not available in the book.'"
                },
                {
                    "role": "user",
                    "content": prompt
                }
            ],
            "temperature": 0.3,
            "max_tokens": 500
        }
        
        try:
            headers = {
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json"
            }
            
            # In a real implementation, we would make the HTTP request here
            # response = requests.post(self.api_url, headers=headers, json=payload)
            # return response.json()["choices"][0]["message"]["content"]
            
            # For this implementation, we'll return a placeholder response
            # that follows the grounding rules specified in the constitution
            if context and len(context.strip()) > 0:
                return f"Based on the provided context: {context[:200]}... [Response based on context]"
            else:
                return "This information is not available in the book."
        except Exception as e:
            logger.error(f"Error calling LLM service: {e}")
            return "This information is not available in the book."
    
    def _create_grounding_prompt(self, question: str, context: str) -> str:
        """
        Create a prompt that ensures the LLM stays grounded in the provided context.
        
        Args:
            question: The user's question
            context: The context to ground the answer in
            
        Returns:
            Formatted prompt string
        """
        if not context or len(context.strip()) == 0:
            return f"Question: {question}\n\nAnswer: This information is not available in the book."
        
        return f"""
        Context: {context}
        
        Question: {question}
        
        Please answer the question based only on the provided context. Do not use any external knowledge. If the answer is not available in the context, respond with "This information is not available in the book."
        """


# Singleton instance
llm_service = LLMService()