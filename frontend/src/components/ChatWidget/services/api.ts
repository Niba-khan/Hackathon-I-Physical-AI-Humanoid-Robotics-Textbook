// frontend/src/components/ChatWidget/services/api.ts

export interface QueryRequest {
  question: string;
  session_id?: string;
  query_mode: 'full-book' | 'selected-text';
  selected_text?: string;
}

export interface QueryResponse {
  response: string;
  session_id: string;
  retrieved_chunks: RetrievedChunk[];
  query_mode: 'full-book' | 'selected-text';
}

export interface RetrievedChunk {
  chunk_id: string;
  source_page: string;
  section_title: string;
}

export interface SessionResponse {
  session_id: string;
  created_at: string;
  query_results: QueryResult[];
}

export interface QueryResult {
  query_text: string;
  response_text: string;
  created_at: string;
}

export interface CreateSessionRequest {
  user_id?: string;
}

export interface CreateSessionResponse {
  session_id: string;
  user_id?: string;
  active: boolean;
  created_at: string;
  updated_at: string;
}

export interface GetSessionResponse {
  session_id: string;
  user_id?: string;
  active: boolean;
  created_at: string;
  updated_at: string;
  metadata: Record<string, any>;
  query_results: QueryResult[];
}

export interface UpdateSessionRequest {
  active?: boolean;
}

const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000/api/v1';

class ApiService {
  static async query(request: QueryRequest): Promise<QueryResponse> {
    try {
      const response = await fetch(`${API_BASE_URL}/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error querying the API:', error);
      throw error;
    }
  }

  static async getSession(sessionId: string): Promise<SessionResponse> {
    try {
      const response = await fetch(`${API_BASE_URL}/sessions/${sessionId}`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        if (response.status === 404) {
          throw new Error('Session not found');
        }
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error fetching session:', error);
      throw error;
    }
  }

  static async ingest(content: string, sourcePage: string, sectionTitle: string): Promise<void> {
    try {
      const response = await fetch(`${API_BASE_URL}/ingest`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content,
          source_page: sourcePage,
          section_title: sectionTitle,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
    } catch (error) {
      console.error('Error ingesting content:', error);
      throw error;
    }
  }

  static async createSession(request: CreateSessionRequest): Promise<CreateSessionResponse> {
    try {
      const response = await fetch(`${API_BASE_URL}/sessions`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error creating session:', error);
      throw error;
    }
  }

  static async updateSession(sessionId: string, request: UpdateSessionRequest): Promise<CreateSessionResponse> {
    try {
      const response = await fetch(`${API_BASE_URL}/sessions/${sessionId}`, {
        method: 'PATCH',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error updating session:', error);
      throw error;
    }
  }
}

export default ApiService;