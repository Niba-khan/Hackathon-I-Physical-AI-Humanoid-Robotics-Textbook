/**
 * Docusaurus Chat Widget Integration Script
 * This script embeds the RAG chat widget into Docusaurus pages
 */

(function() {
    // Configuration - can be overridden by users
    const CONFIG = {
        // API endpoint configuration
        API_BASE_URL: window.CHATBOT_CONFIG?.API_BASE_URL || 'http://localhost:8000/api/v1',

        // Widget positioning
        position: window.CHATBOT_CONFIG?.position || 'bottom-right',

        // Widget appearance
        widgetTitle: window.CHATBOT_CONFIG?.widgetTitle || 'Book Assistant',

        // Initial mode
        defaultMode: window.CHATBOT_CONFIG?.defaultMode || 'full-book'  // 'full-book' or 'selected-text'
    };

    // Create the widget container and load the React app
    function initializeChatWidget() {
        // Create the container for the chat widget
        const widgetContainer = document.createElement('div');
        widgetContainer.id = 'chatbot-container';
        widgetContainer.style.cssText = `
            position: fixed;
            bottom: 20px;
            right: 20px;
            z-index: 9999;
            width: 400px;
            height: 500px;
            box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
            border-radius: 8px;
            display: none;
        `;

        // Create toggle button
        const toggleButton = document.createElement('button');
        toggleButton.id = 'chatbot-toggle';
        toggleButton.innerHTML = '📖';
        toggleButton.style.cssText = `
            position: fixed;
            bottom: 20px;
            right: 20px;
            z-index: 10000;
            width: 60px;
            height: 60px;
            border-radius: 50%;
            background: #4a6cf7;
            color: white;
            border: none;
            font-size: 24px;
            cursor: pointer;
            box-shadow: 0 2px 10px rgba(0, 0, 0, 0.2);
            display: flex;
            align-items: center;
            justify-content: center;
        `;

        // Add toggle functionality
        let isOpen = false;
        toggleButton.addEventListener('click', function() {
            if (isOpen) {
                widgetContainer.style.display = 'none';
                toggleButton.innerHTML = '📖';
            } else {
                widgetContainer.style.display = 'block';
                toggleButton.innerHTML = '×';
            }
            isOpen = !isOpen;
        });

        // Add event listener for text selection
        document.addEventListener('mouseup', function() {
            const selectedText = window.getSelection().toString().trim();
            if (selectedText) {
                // Store selected text in a global variable for the chat widget to access
                window.CHATBOT_SELECTED_TEXT = selectedText;
            }
        });

        // Append elements to the body
        document.body.appendChild(widgetContainer);
        document.body.appendChild(toggleButton);

        // Load the React chat widget app
        loadChatWidgetApp();
    }

    // Load the React app into the container
    function loadChatWidgetApp() {
        // Create a container for the React app
        const container = document.getElementById('chatbot-container');

        // Create the root element for React
        const reactRoot = document.createElement('div');
        reactRoot.id = 'chat-widget-root';
        reactRoot.style.width = '100%';
        reactRoot.style.height = '100%';

        container.appendChild(reactRoot);

        // Dynamically load React and ReactDOM if not already present
        if (!window.React || !window.ReactDOM) {
            const reactScript = document.createElement('script');
            reactScript.src = 'https://unpkg.com/react@18/umd/react.production.min.js';
            reactScript.onload = function() {
                const reactDOMScript = document.createElement('script');
                reactDOMScript.src = 'https://unpkg.com/react-dom@18/umd/react-dom.production.min.js';
                reactDOMScript.onload = function() {
                    loadAndRenderChatWidget();
                };
                document.head.appendChild(reactDOMScript);
            };
            document.head.appendChild(reactScript);
        } else {
            loadAndRenderChatWidget();
        }
    }

    // Function to load and render the ChatWidget component
    function loadAndRenderChatWidget() {
        // Create script element to load the ChatWidget bundle
        const script = document.createElement('script');
        script.type = 'module';

        // This would be the path to the built ChatWidget component
        // For now we'll load the component using dynamic import approach
        script.textContent = `
            // Create a proper React component for the chat widget that connects to the backend
            (async () => {
                try {
                    // Define a proper ChatWidget component that connects to the backend
                    const ChatWidgetComponent = () => {
                        const [messages, setMessages] = React.useState([]);
                        const [inputValue, setInputValue] = React.useState('');
                        const [isLoading, setIsLoading] = React.useState(false);
                        const [queryMode, setQueryMode] = React.useState('full-book');
                        const [sessionId, setSessionId] = React.useState(null);

                        // Initialize with a welcome message and create a session
                        React.useEffect(() => {
                            setMessages([
                                {
                                    id: '1',
                                    text: 'Hello! I\\'m your AI assistant for this book. How can I help you today?',
                                    isUser: false,
                                    timestamp: new Date()
                                }
                            ]);

                            // Create a new session with the backend
                            fetch(CONFIG.API_BASE_URL + '/sessions', {
                                method: 'POST',
                                headers: {
                                    'Content-Type': 'application/json',
                                },
                                body: JSON.stringify({})
                            })
                            .then(response => response.json())
                            .then(data => {
                                setSessionId(data.session_id);
                            })
                            .catch(error => {
                                console.error('Error creating session:', error);
                            });
                        }, []);

                        const handleSend = async () => {
                            if (!inputValue.trim() || isLoading) return;

                            // Add user message
                            const userMessage = {
                                id: Date.now().toString(),
                                text: inputValue,
                                isUser: true,
                                timestamp: new Date()
                            };

                            setMessages(prev => [...prev, userMessage]);
                            setInputValue('');
                            setIsLoading(true);

                            try {
                                // Prepare the query request
                                const queryRequest = {
                                    question: inputValue,
                                    query_mode: queryMode
                                };

                                // Add session ID if available
                                if (sessionId) {
                                    queryRequest.session_id = sessionId;
                                }

                                // Add selected text if in selected-text mode and text has been selected
                                if (queryMode === 'selected-text' && window.CHATBOT_SELECTED_TEXT) {
                                    queryRequest.selected_text = window.CHATBOT_SELECTED_TEXT;
                                }

                                // Send the query to the backend
                                const response = await fetch(CONFIG.API_BASE_URL + '/query', {
                                    method: 'POST',
                                    headers: {
                                        'Content-Type': 'application/json',
                                    },
                                    body: JSON.stringify(queryRequest)
                                });

                                if (!response.ok) {
                                    throw new Error('Network response was not ok');
                                }

                                const data = await response.json();

                                // Update session ID if it was returned in the response
                                if (data.session_id && !sessionId) {
                                    setSessionId(data.session_id);
                                }

                                const botMessage = {
                                    id: (Date.now() + 1).toString(),
                                    text: data.response,
                                    isUser: false,
                                    timestamp: new Date(),
                                    query_mode: data.query_mode
                                };

                                setMessages(prev => [...prev, botMessage]);
                            } catch (error) {
                                console.error('Error sending query:', error);

                                const errorMessage = {
                                    id: (Date.now() + 1).toString(),
                                    text: 'Sorry, I encountered an error processing your request. Please try again.',
                                    isUser: false,
                                    timestamp: new Date()
                                };

                                setMessages(prev => [...prev, errorMessage]);
                            } finally {
                                setIsLoading(false);
                            }
                        };

                        const handleKeyDown = (e) => {
                            if (e.key === 'Enter' && !e.shiftKey) {
                                e.preventDefault();
                                handleSend();
                            }
                        };

                        return React.createElement('div', {style: {height: '100%', display: 'flex', flexDirection: 'column'}},
                            // Header
                            React.createElement('div', {style: {background: '#4a6cf7', color: 'white', padding: '10px', textAlign: 'center'}},
                                React.createElement('h3', null, 'Book Assistant')
                            ),
                            // Messages container
                            React.createElement('div', {style: {flex: 1, overflowY: 'auto', padding: '10px', background: '#f9f9f9'}},
                                messages.map(msg =>
                                    React.createElement('div', {
                                        key: msg.id,
                                        style: {
                                            margin: '10px 0',
                                            padding: '8px',
                                            borderRadius: '8px',
                                            maxWidth: '80%',
                                            backgroundColor: msg.isUser ? '#4a6cf7' : '#e9ecef',
                                            color: msg.isUser ? 'white' : '#333',
                                            marginLeft: msg.isUser ? 'auto' : '0',
                                            wordWrap: 'break-word'
                                        }
                                    },
                                        React.createElement('div', null, msg.text),
                                        msg.query_mode && React.createElement('div', {style: {fontSize: '0.75rem', color: '#888', textAlign: 'right', marginTop: '3px'}},
                                            msg.query_mode === 'full-book' ? '📚 Book Mode' : '🔍 Text Mode'
                                        )
                                    )
                                ),
                                isLoading && React.createElement('div', {style: {margin: '10px 0', padding: '8px'}},
                                    React.createElement('div', {style: {display: 'flex', alignItems: 'center', gap: '4px'}},
                                        React.createElement('span', {style: {width: '8px', height: '8px', backgroundColor: '#888', borderRadius: '50%'}}, ''),
                                        React.createElement('span', {style: {width: '8px', height: '8px', backgroundColor: '#888', borderRadius: '50%'}}, ''),
                                        React.createElement('span', {style: {width: '8px', height: '8px', backgroundColor: '#888', borderRadius: '50%'}}, '')
                                    )
                                )
                            ),
                            // Mode selector
                            React.createElement('div', {style: {display: 'flex', borderBottom: '1px solid #eee', backgroundColor: '#f5f5f5'}},
                                React.createElement('button', {
                                    onClick: () => setQueryMode('full-book'),
                                    style: {
                                        flex: 1,
                                        padding: '10px',
                                        border: 'none',
                                        backgroundColor: queryMode === 'full-book' ? '#4a6cf7' : 'transparent',
                                        color: queryMode === 'full-book' ? 'white' : '#333',
                                        cursor: 'pointer',
                                        fontWeight: queryMode === 'full-book' ? 'bold' : 'normal'
                                    }
                                }, 'Full Book'),
                                React.createElement('button', {
                                    onClick: () => setQueryMode('selected-text'),
                                    style: {
                                        flex: 1,
                                        padding: '10px',
                                        border: 'none',
                                        backgroundColor: queryMode === 'selected-text' ? '#4a6cf7' : 'transparent',
                                        color: queryMode === 'selected-text' ? 'white' : '#333',
                                        cursor: 'pointer',
                                        fontWeight: queryMode === 'selected-text' ? 'bold' : 'normal'
                                    }
                                }, 'Selected Text')
                            ),
                            // Input area
                            React.createElement('div', {style: {padding: '10px', background: 'white', borderTop: '1px solid #eee'}},
                                React.createElement('div', {style: {display: 'flex'}},
                                    React.createElement('textarea', {
                                        value: inputValue,
                                        onChange: (e) => setInputValue(e.target.value),
                                        onKeyDown: handleKeyDown,
                                        placeholder: queryMode === 'selected-text'
                                            ? "Ask a question about the selected text..."
                                            : "Ask a question about the book...",
                                        rows: 2,
                                        style: {
                                            flex: 1,
                                            padding: '8px',
                                            border: '1px solid #ddd',
                                            borderRadius: '4px',
                                            resize: 'none'
                                        },
                                        disabled: isLoading
                                    }),
                                    React.createElement('button', {
                                        onClick: handleSend,
                                        disabled: isLoading || !inputValue.trim(),
                                        style: {
                                            padding: '8px 16px',
                                            marginLeft: '5px',
                                            backgroundColor: '#4a6cf7',
                                            color: 'white',
                                            border: 'none',
                                            borderRadius: '4px',
                                            cursor: isLoading || !inputValue.trim() ? 'not-allowed' : 'pointer'
                                        }
                                    }, 'Send')
                                ),
                                queryMode === 'selected-text' && window.CHATBOT_SELECTED_TEXT &&
                                    React.createElement('div', {style: {marginTop: '8px', padding: '8px', backgroundColor: '#e9f7fe', border: '1px solid #b3d9ff', borderRadius: '4px', fontSize: '0.8rem'}},
                                        React.createElement('strong', null, 'Selected: '),
                                        window.CHATBOT_SELECTED_TEXT.length > 100
                                            ? window.CHATBOT_SELECTED_TEXT.substring(0, 100) + '...'
                                            : window.CHATBOT_SELECTED_TEXT
                                    )
                            )
                        );
                    };

                    // Render the component
                    ReactDOM.render(React.createElement(ChatWidgetComponent), document.getElementById('chat-widget-root'));
                } catch (error) {
                    console.error('Error loading chat widget:', error);

                    // Fallback to basic HTML implementation
                    const container = document.getElementById('chat-widget-root');
                    container.innerHTML = \`
                        <div style="height: 100%; display: flex; flex-direction: column;">
                            <div style="background: #4a6cf7; color: white; padding: 10px; text-align: center;">
                                <h3>\${CONFIG.widgetTitle}</h3>
                            </div>
                            <div id="messages" style="flex: 1; overflow-y: auto; padding: 10px; background: #f9f9f9;">
                                <div class="message bot-message" style="margin: 10px 0; padding: 8px; border-radius: 8px; max-width: 80%; background: #e9ecef; color: #333;">
                                    Hello! I'm your AI assistant for this book. How can I help you today?
                                </div>
                            </div>
                            <div style="padding: 10px; background: white; border-top: 1px solid #eee;">
                                <div style="display: flex;">
                                    <textarea id="user-input" placeholder="Ask a question about the book..." rows="2" style="flex: 1; padding: 8px; border: 1px solid #ddd; border-radius: 4px; resize: none;"></textarea>
                                    <button id="send-btn" style="padding: 8px 16px; margin-left: 5px; background: #4a6cf7; color: white; border: none; border-radius: 4px; cursor: pointer;">Send</button>
                                </div>
                            </div>
                        </div>
                        <script>
                            document.getElementById('send-btn').addEventListener('click', sendMessage);
                            document.getElementById('user-input').addEventListener('keydown', function(e) {
                                if (e.key === 'Enter' && !e.shiftKey) {
                                    e.preventDefault();
                                    sendMessage();
                                }
                            });

                            function sendMessage() {
                                const input = document.getElementById('user-input');
                                const message = input.value.trim();
                                if (!message) return;

                                // Add user message to chat
                                const messagesDiv = document.getElementById('messages');
                                const userMsg = document.createElement('div');
                                userMsg.className = 'message user-message';
                                userMsg.style.cssText = "margin: 10px 0; padding: 8px; border-radius: 8px; max-width: 80%; background: #4a6cf7; color: white; margin-left: auto;";
                                userMsg.textContent = message;
                                messagesDiv.appendChild(userMsg);

                                // Add bot response
                                const botMsg = document.createElement('div');
                                botMsg.className = 'message bot-message';
                                botMsg.style.cssText = "margin: 10px 0; padding: 8px; border-radius: 8px; max-width: 80%; background: #e9ecef; color: #333;";
                                botMsg.textContent = 'This is a demo response. In the full implementation, this would be connected to the backend API.';
                                messagesDiv.appendChild(botMsg);

                                // Clear input
                                input.value = '';

                                // Scroll to bottom
                                messagesDiv.scrollTop = messagesDiv.scrollHeight;
                            }
                        <\/script>
                    \`;
                }
            })();
        `;

        document.head.appendChild(script);
    }

    // Initialize the chat widget when the DOM is loaded
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', initializeChatWidget);
    } else {
        initializeChatWidget();
    }
})();