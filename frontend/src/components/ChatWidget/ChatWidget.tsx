import React, { useState, useEffect } from 'react';
import './styles.css';
import ApiService, { QueryRequest } from './services/api';
import InputArea from './InputArea';
import ChatMessage from './ChatMessage';

interface Message {
  id: string;
  text: string;
  isUser: boolean;
  timestamp: Date;
  queryMode?: 'full-book' | 'selected-text';
  sources?: Array<{
    chunk_id: string;
    source_page: string;
    section_title: string;
  }>;
}

type QueryMode = 'full-book' | 'selected-text';

const ChatWidget: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [queryMode, setQueryMode] = useState<QueryMode>('full-book');
  const [selectedText, setSelectedText] = useState<string>('');
  const [sessionId, setSessionId] = useState<string | null>(null);

  // Initialize session and with a welcome message
  useEffect(() => {
    const initializeSession = async () => {
      try {
        // Create a new session
        const sessionResponse = await ApiService.createSession({});
        setSessionId(sessionResponse.session_id);

        setMessages([
          {
            id: '1',
            text: 'Hello! I\'m your AI assistant for this book. How can I help you today?',
            isUser: false,
            timestamp: new Date(),
          }
        ]);
      } catch (error) {
        console.error('Failed to create session:', error);
        // Fallback to showing the welcome message without a session
        setMessages([
          {
            id: '1',
            text: 'Hello! I\'m your AI assistant for this book. How can I help you today?',
            isUser: false,
            timestamp: new Date(),
          }
        ]);
      }
    };

    initializeSession();
  }, []);

  const handleSend = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      text: inputValue,
      isUser: true,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Prepare the query request
      const queryRequest: QueryRequest = {
        question: inputValue,
        query_mode: queryMode,
      };

      // Add session ID if available
      if (sessionId) {
        queryRequest.session_id = sessionId;
      }

      // Add selected text if in selected-text mode
      if (queryMode === 'selected-text' && selectedText) {
        queryRequest.selected_text = selectedText;
      }

      // Call the backend API via ApiService
      const response = await ApiService.query(queryRequest);

      // Update session ID if it was returned in the response
      if (response.session_id && !sessionId) {
        setSessionId(response.session_id);
      }

      const botMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: response.response,
        isUser: false,
        timestamp: new Date(),
        queryMode: response.query_mode,
        sources: response.retrieved_chunks
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        isUser: false,
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const toggleQueryMode = () => {
    setQueryMode(prev => prev === 'full-book' ? 'selected-text' : 'full-book');
  };

  return (
    <div className="chat-widget">
      <div className="chat-header">
        <h3>Book Assistant</h3>
      </div>

      <div className="chat-messages">
        {messages.map((message) => (
          <ChatMessage
            key={message.id}
            id={message.id}
            text={message.text}
            isUser={message.isUser}
            timestamp={message.timestamp}
          />
        ))}

        {isLoading && (
          <div className="message bot-message">
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
      </div>

      <div className="mode-selector">
        <button
          className={`mode-button ${queryMode === 'full-book' ? 'active' : ''}`}
          onClick={() => setQueryMode('full-book')}
        >
          Full Book
        </button>
        <button
          className={`mode-button ${queryMode === 'selected-text' ? 'active' : ''}`}
          onClick={() => {
            setQueryMode('selected-text');
            // In a real implementation, we would get the selected text from the page
            // For demo purposes, we'll use placeholder text
            setSelectedText('This is the selected text from the page. In the actual implementation, this would be text selected by the user.');
          }}
        >
          Selected Text
        </button>
      </div>

      <div className="chat-input-area">
        <InputArea
          onSend={handleSend}
          isLoading={isLoading}
          queryMode={queryMode}
          selectedText={selectedText}
        />
      </div>
    </div>
  );
};

export default ChatWidget;