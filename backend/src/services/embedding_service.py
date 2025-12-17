from sentence_transformers import SentenceTransformer
import numpy as np
from typing import List, Union
import logging


logger = logging.getLogger(__name__)


class EmbeddingService:
    def __init__(self, model_name: str = "all-MiniLM-L6-v2"):
        """
        Initialize the embedding service with a pre-trained model.
        
        Args:
            model_name: Name of the sentence transformer model to use
        """
        self.model = SentenceTransformer(model_name)
        logger.info(f"Embedding service initialized with model: {model_name}")
    
    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.
        
        Args:
            text: Input text to embed
            
        Returns:
            List of floats representing the embedding
        """
        try:
            embedding = self.model.encode([text])
            # Convert to list of floats for JSON serialization
            return embedding[0].tolist()
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise
    
    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts.
        
        Args:
            texts: List of input texts to embed
            
        Returns:
            List of embeddings (each embedding is a list of floats)
        """
        try:
            embeddings = self.model.encode(texts)
            # Convert to list of lists of floats for JSON serialization
            return [embedding.tolist() for embedding in embeddings]
        except Exception as e:
            logger.error(f"Error generating embeddings: {e}")
            raise
    
    def get_embedding_dimension(self) -> int:
        """
        Get the dimension of the embeddings produced by the model.
        
        Returns:
            Dimension of the embeddings
        """
        # Create a dummy sentence to get embedding dimension
        dummy_embedding = self.model.encode(["dummy sentence"])
        return len(dummy_embedding[0])