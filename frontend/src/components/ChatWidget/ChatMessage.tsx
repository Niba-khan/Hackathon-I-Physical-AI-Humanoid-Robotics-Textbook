import React from 'react';

interface ChatMessageProps {
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

const ChatMessage: React.FC<ChatMessageProps> = ({
  id,
  text,
  isUser,
  timestamp,
  queryMode,
  sources
}) => {
  return (
    <div className={`message ${isUser ? 'user-message' : 'bot-message'}`}>
      <div className="message-content">{text}</div>
      <div className="message-meta">
        <div className="message-timestamp">
          {timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
        </div>
        {queryMode && (
          <div className="message-query-mode" title={`Query mode: ${queryMode}`}>
            {queryMode === 'full-book' ? '📚' : '🔍'}
          </div>
        )}
      </div>
      {sources && sources.length > 0 && (
        <div className="message-sources">
          <details>
            <summary>Sources</summary>
            <ul>
              {sources.map((source, index) => (
                <li key={index}>
                  <a href={source.source_page} target="_blank" rel="noopener noreferrer">
                    {source.section_title || source.source_page}
                  </a>
                </li>
              ))}
            </ul>
          </details>
        </div>
      )}
    </div>
  );
};

export default ChatMessage;