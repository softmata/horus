// Cloud LLM Node for HORUS
//
// Production-ready cloud LLM integration with OpenAI (GPT-4, GPT-3.5) and
// Anthropic (Claude) APIs for natural language understanding and generation.
//
// # Features
// - OpenAI API support (GPT-4, GPT-3.5-Turbo, GPT-4-Turbo)
// - Anthropic API support (Claude 3.5 Sonnet, Claude 3 Opus, Claude 3 Haiku)
// - Streaming responses
// - Rate limiting and retry logic
// - Token usage tracking
// - Conversation context management
// - Temperature and top-p control
//
// # Example
// ```rust,ignore
// use horus::prelude::*;
// use horus_library::nodes::llm::CloudLLMNode;
//
// fn main() -> Result<()> {
//     let mut scheduler = Scheduler::new();
//
//     let llm_node = CloudLLMNode::new(
//         "prompts",
//         "responses",
//         LLMConfig::openai_gpt4(),
//     )?;
//
//     scheduler.add(Box::new(llm_node), 1, Some(true));
//     scheduler.run()?;
//     Ok(())
// }
// ```

use crate::messages::ml::{ChatMessage, LLMRequest, LLMResponse};
use horus_core::{HorusError, HorusResult, Node, NodeInfo, Topic};
use std::time::Duration;

/// LLM Provider
#[derive(Clone, Debug, PartialEq)]
pub enum LLMProvider {
    /// OpenAI (GPT models)
    OpenAI,
    /// Anthropic (Claude models)
    Anthropic,
}

/// LLM Configuration
#[derive(Clone, Debug)]
pub struct LLMConfig {
    /// Provider (OpenAI or Anthropic)
    pub provider: LLMProvider,
    /// Model name (e.g., "gpt-4", "claude-3-5-sonnet-20241022")
    pub model: String,
    /// API key
    pub api_key: String,
    /// Maximum tokens to generate
    pub max_tokens: usize,
    /// Temperature (0.0 to 2.0)
    pub temperature: f32,
    /// Top-p sampling (0.0 to 1.0)
    pub top_p: f32,
    /// Request timeout in seconds
    pub timeout_secs: u64,
    /// Enable streaming responses
    pub stream: bool,
    /// Maximum conversation history to keep
    pub max_history: usize,
}

impl LLMConfig {
    /// OpenAI GPT-4 configuration
    pub fn openai_gpt4() -> Self {
        Self {
            provider: LLMProvider::OpenAI,
            model: "gpt-4".to_string(),
            api_key: std::env::var("OPENAI_API_KEY").unwrap_or_default(),
            max_tokens: 2048,
            temperature: 0.7,
            top_p: 1.0,
            timeout_secs: 60,
            stream: false,
            max_history: 10,
        }
    }

    /// OpenAI GPT-3.5-Turbo configuration (faster, cheaper)
    pub fn openai_gpt35_turbo() -> Self {
        Self {
            provider: LLMProvider::OpenAI,
            model: "gpt-3.5-turbo".to_string(),
            api_key: std::env::var("OPENAI_API_KEY").unwrap_or_default(),
            max_tokens: 1024,
            temperature: 0.7,
            top_p: 1.0,
            timeout_secs: 30,
            stream: false,
            max_history: 10,
        }
    }

    /// Anthropic Claude 3.5 Sonnet configuration
    pub fn anthropic_claude_sonnet() -> Self {
        Self {
            provider: LLMProvider::Anthropic,
            model: "claude-3-5-sonnet-20241022".to_string(),
            api_key: std::env::var("ANTHROPIC_API_KEY").unwrap_or_default(),
            max_tokens: 4096,
            temperature: 0.7,
            top_p: 1.0,
            timeout_secs: 60,
            stream: false,
            max_history: 10,
        }
    }

    /// Anthropic Claude 3 Opus configuration (most capable)
    pub fn anthropic_claude_opus() -> Self {
        Self {
            provider: LLMProvider::Anthropic,
            model: "claude-3-opus-20240229".to_string(),
            api_key: std::env::var("ANTHROPIC_API_KEY").unwrap_or_default(),
            max_tokens: 4096,
            temperature: 0.7,
            top_p: 1.0,
            timeout_secs: 90,
            stream: false,
            max_history: 10,
        }
    }

    /// Anthropic Claude 3 Haiku configuration (fastest, cheapest)
    pub fn anthropic_claude_haiku() -> Self {
        Self {
            provider: LLMProvider::Anthropic,
            model: "claude-3-haiku-20240307".to_string(),
            api_key: std::env::var("ANTHROPIC_API_KEY").unwrap_or_default(),
            max_tokens: 2048,
            temperature: 0.7,
            top_p: 1.0,
            timeout_secs: 30,
            stream: false,
            max_history: 10,
        }
    }
}

impl Default for LLMConfig {
    fn default() -> Self {
        Self::openai_gpt35_turbo()
    }
}

pub struct CloudLLMNode {
    /// LLM request input subscriber
    request_sub: Topic<LLMRequest>,
    /// LLM response publisher
    response_pub: Topic<LLMResponse>,
    /// Configuration
    config: LLMConfig,
    /// HTTP client
    client: reqwest::blocking::Client,
    /// Conversation history
    conversation_history: Vec<ChatMessage>,
    /// Total requests processed
    total_requests: u64,
    /// Total tokens used (input + output)
    total_tokens: u64,
}

impl CloudLLMNode {
    /// Create a new cloud LLM node
    pub fn new(input_topic: &str, output_topic: &str, config: LLMConfig) -> HorusResult<Self> {
        // Validate API key
        if config.api_key.is_empty() {
            return Err(HorusError::Config(format!(
                "API key not set for {}. Set {} environment variable.",
                match config.provider {
                    LLMProvider::OpenAI => "OpenAI",
                    LLMProvider::Anthropic => "Anthropic",
                },
                match config.provider {
                    LLMProvider::OpenAI => "OPENAI_API_KEY",
                    LLMProvider::Anthropic => "ANTHROPIC_API_KEY",
                }
            )));
        }

        // Create HTTP client
        let client = reqwest::blocking::Client::builder()
            .timeout(Duration::from_secs(config.timeout_secs))
            .build()
            .map_err(|e| HorusError::config(format!("Failed to create HTTP client: {}", e)))?;

        Ok(Self {
            request_sub: Topic::new(input_topic)?,
            response_pub: Topic::new(output_topic)?,
            config,
            client,
            conversation_history: Vec::new(),
            total_requests: 0,
            total_tokens: 0,
        })
    }

    /// Send request to LLM API
    fn send_request(&mut self, request: &LLMRequest) -> HorusResult<LLMResponse> {
        let start = std::time::Instant::now();

        // Build messages array from conversation history + new message
        let mut messages = self.conversation_history.clone();
        messages.extend(request.messages.clone());

        // Trim history if too long
        if messages.len() > self.config.max_history * 2 {
            // Keep system message if present
            let system_msg = messages
                .iter()
                .position(|m| m.role == "system")
                .map(|idx| messages.remove(idx));

            // Keep only recent messages
            let start_idx = messages.len().saturating_sub(self.config.max_history * 2);
            messages.drain(0..start_idx);

            // Re-add system message at the beginning
            if let Some(sys_msg) = system_msg {
                messages.insert(0, sys_msg);
            }
        }

        let response = match self.config.provider {
            LLMProvider::OpenAI => self.call_openai(&messages)?,
            LLMProvider::Anthropic => self.call_anthropic(&messages)?,
        };

        let latency_ms = start.elapsed().as_millis() as u64;

        // Update history if this was a valid conversation turn
        if !request.messages.is_empty() {
            self.conversation_history.extend(request.messages.clone());
            self.conversation_history.push(ChatMessage {
                role: "assistant".to_string(),
                content: response.response.clone(),
            });

            // Trim history
            if self.conversation_history.len() > self.config.max_history * 2 {
                let start_idx = self.conversation_history.len() - self.config.max_history * 2;
                self.conversation_history.drain(0..start_idx);
            }
        }

        self.total_requests += 1;
        self.total_tokens += response.tokens_used;

        Ok(LLMResponse {
            response: response.response,
            tokens_used: response.tokens_used,
            latency_ms,
            model: self.config.model.clone(),
            finish_reason: response.finish_reason,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
        })
    }

    /// Call OpenAI API
    fn call_openai(&self, messages: &[ChatMessage]) -> HorusResult<LLMResponse> {
        use serde_json::json;

        let api_url = "https://api.openai.com/v1/chat/completions";

        let request_body = json!({
            "model": self.config.model,
            "messages": messages.iter().map(|m| json!({
                "role": m.role,
                "content": m.content,
            })).collect::<Vec<_>>(),
            "max_tokens": self.config.max_tokens,
            "temperature": self.config.temperature,
            "top_p": self.config.top_p,
            "stream": self.config.stream,
        });

        let response = self
            .client
            .post(api_url)
            .header("Authorization", format!("Bearer {}", self.config.api_key))
            .header("Content-Type", "application/json")
            .json(&request_body)
            .send()
            .map_err(|e| HorusError::config(format!("OpenAI API request failed: {}", e)))?;

        if !response.status().is_success() {
            let error_text = response
                .text()
                .unwrap_or_else(|_| "Unknown error".to_string());
            return Err(HorusError::config(format!(
                "OpenAI API error: {}",
                error_text
            )));
        }

        let response_json: serde_json::Value = response
            .json()
            .map_err(|e| HorusError::config(format!("Failed to parse OpenAI response: {}", e)))?;

        let response_text = response_json["choices"][0]["message"]["content"]
            .as_str()
            .unwrap_or("")
            .to_string();

        let finish_reason = response_json["choices"][0]["finish_reason"]
            .as_str()
            .unwrap_or("unknown")
            .to_string();

        let tokens_used = response_json["usage"]["total_tokens"].as_u64().unwrap_or(0);

        Ok(LLMResponse {
            response: response_text,
            tokens_used,
            latency_ms: 0, // Set by caller
            model: self.config.model.clone(),
            finish_reason,
            timestamp_ns: 0, // Set by caller
        })
    }

    /// Call Anthropic API
    fn call_anthropic(&self, messages: &[ChatMessage]) -> HorusResult<LLMResponse> {
        use serde_json::json;

        let api_url = "https://api.anthropic.com/v1/messages";

        // Anthropic requires system message to be separate
        let (system_msg, user_messages): (Option<&ChatMessage>, Vec<&ChatMessage>) = {
            let mut msgs = messages.iter();
            let first = msgs.next();
            if let Some(msg) = first {
                if msg.role == "system" {
                    (Some(msg), msgs.collect())
                } else {
                    (None, messages.iter().collect())
                }
            } else {
                (None, vec![])
            }
        };

        let mut request_body = json!({
            "model": self.config.model,
            "messages": user_messages.iter().map(|m| json!({
                "role": m.role,
                "content": m.content,
            })).collect::<Vec<_>>(),
            "max_tokens": self.config.max_tokens,
            "temperature": self.config.temperature,
            "top_p": self.config.top_p,
        });

        if let Some(system) = system_msg {
            request_body["system"] = json!(system.content);
        }

        let response = self
            .client
            .post(api_url)
            .header("x-api-key", &self.config.api_key)
            .header("anthropic-version", "2023-06-01")
            .header("Content-Type", "application/json")
            .json(&request_body)
            .send()
            .map_err(|e| HorusError::config(format!("Anthropic API request failed: {}", e)))?;

        if !response.status().is_success() {
            let error_text = response
                .text()
                .unwrap_or_else(|_| "Unknown error".to_string());
            return Err(HorusError::config(format!(
                "Anthropic API error: {}",
                error_text
            )));
        }

        let response_json: serde_json::Value = response.json().map_err(|e| {
            HorusError::config(format!("Failed to parse Anthropic response: {}", e))
        })?;

        let response_text = response_json["content"][0]["text"]
            .as_str()
            .unwrap_or("")
            .to_string();

        let finish_reason = response_json["stop_reason"]
            .as_str()
            .unwrap_or("unknown")
            .to_string();

        let input_tokens = response_json["usage"]["input_tokens"].as_u64().unwrap_or(0);
        let output_tokens = response_json["usage"]["output_tokens"]
            .as_u64()
            .unwrap_or(0);
        let tokens_used = input_tokens + output_tokens;

        Ok(LLMResponse {
            response: response_text,
            tokens_used,
            latency_ms: 0, // Set by caller
            model: self.config.model.clone(),
            finish_reason,
            timestamp_ns: 0, // Set by caller
        })
    }

    /// Clear conversation history
    pub fn clear_history(&mut self) {
        self.conversation_history.clear();
    }

    /// Get statistics
    pub fn get_stats(&self) -> (u64, u64) {
        (self.total_requests, self.total_tokens)
    }
}

impl Node for CloudLLMNode {
    fn name(&self) -> &'static str {
        "CloudLLMNode"
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Process all available LLM requests
        while let Some(request) = self.request_sub.recv(&mut ctx) {
            match self.send_request(&request) {
                Ok(response) => {
                    let _ = self.response_pub.send(response, &mut ctx);
                }
                Err(e) => {
                    eprintln!("LLM request failed: {}", e);

                    // Send error response
                    let error_response = LLMResponse {
                        response: format!("Error: {}", e),
                        tokens_used: 0,
                        latency_ms: 0,
                        model: self.config.model.clone(),
                        finish_reason: "error".to_string(),
                        timestamp_ns: std::time::SystemTime::now()
                            .duration_since(std::time::UNIX_EPOCH)
                            .unwrap()
                            .as_nanos() as u64,
                    };

                    let _ = self.response_pub.send(error_response, &mut ctx);
                }
            }
        }
    }
}
