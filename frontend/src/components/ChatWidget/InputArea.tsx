import React, { useState } from 'react';

interface InputAreaProps {
  onSend: (message: string) => void;
  isLoading: boolean;
  queryMode: 'full-book' | 'selected-text';
  selectedText?: string;
}

const InputArea: React.FC<InputAreaProps> = ({ onSend, isLoading, queryMode, selectedText }) => {
  const [inputValue, setInputValue] = useState('');

  const handleSend = () => {
    if (inputValue.trim() && !isLoading) {
      onSend(inputValue);
      setInputValue('');
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <div className="chat-input-area">
      <textarea
        value={inputValue}
        onChange={(e) => setInputValue(e.target.value)}
        onKeyDown={handleKeyDown}
        placeholder={queryMode === 'selected-text'
          ? "Ask a question about the selected text..."
          : "Ask a question about the book..."}
        disabled={isLoading}
        rows={2}
      />
      <button
        onClick={handleSend}
        disabled={isLoading || !inputValue.trim()}
        className="send-button"
      >
        Send
      </button>

      {queryMode === 'selected-text' && selectedText && (
        <div className="selected-text-preview">
          <p><strong>Selected Text:</strong></p>
          <p>{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}</p>
        </div>
      )}
    </div>
  );
};

export default InputArea;