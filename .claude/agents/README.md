# Claude Code Custom Agents

This directory contains custom Claude Code agents designed specifically for the Physical AI & Humanoid Robotics project.

## What Are Custom Agents?

Custom agents are specialized AI assistants that have domain-specific knowledge, capabilities, and tools. They extend Claude Code's functionality for project-specific tasks.

---

## Available Agents

### 1. `book-rag-helper` - Robotics Content Assistant

**Purpose**: Specialized agent for helping users navigate and understand the humanoid robotics book content.

**Configuration**: `.claude/agents/book-rag-helper.json`

**Capabilities**:
- Answer questions about ROS 2, Gazebo, Isaac Sim, and VLA models from book content
- Help users find relevant chapters based on their learning goals
- Provide hardware-specific recommendations (GPU/Jetson/Cloud)
- Explain complex robotics concepts with examples from the book

**Tools Available**:
- `read` - Read book content files
- `grep` - Search for specific robotics topics
- `glob` - Find relevant chapters by pattern
- `bash` - Run verification commands

**When to Use**:
- User asks: "Where can I learn about ROS 2 topics?"
- User asks: "Can I run Isaac Sim on my hardware?"
- User asks: "How do I implement VSLAM?"
- User needs chapter recommendations based on their skill level

**Example Usage**:

```bash
# In Claude Code, reference this agent:
@book-rag-helper How do I start learning ROS 2 with a Jetson Orin Nano?
```

**Example Interactions**:

**Input**: "How do I start learning ROS 2?"
**Output**: "Start with Module 1 (03-module-1-ros2.mdx). It covers ROS 2 fundamentals including nodes, topics, services, and actions. If you have a GPU workstation, you can run examples locally. For Jetson users, focus on the ARM64 installation section."

**Input**: "Can I run Isaac Sim on Mac?"
**Output**: "No, Isaac Sim requires an NVIDIA RTX GPU and won't work on Mac. See Module 3 (05-module-3-nvidia-isaac.mdx) for alternatives: use Omniverse Cloud for cloud-based Isaac Sim, or stick with Gazebo (Module 2) which works on Mac via Docker."

---

### 2. `hardware-advisor` - Hardware Recommendation Assistant

**Purpose**: Specialized agent for hardware purchasing decisions and budget planning.

**Configuration**: `.claude/agents/hardware-advisor.json`

**Capabilities**:
- Recommend hardware setups based on budget constraints ($0-$500, $500-$2000, $2000+)
- Provide cost breakdowns for GPU workstations, Jetson edge devices, and cloud alternatives
- Consider user's existing hardware and recommend complementary purchases
- Advise students, educators, and hobbyists with role-appropriate recommendations
- Compare local GPU vs Jetson vs Cloud/Mac approaches with pros/cons

**Tools Available**:
- `read` - Read hardware requirements documentation
- `grep` - Search for specific hardware specs
- `bash` - Run cost calculations

**When to Use**:
- User asks: "What hardware do I need for this course?"
- User asks: "I have $1000, what should I buy?"
- User asks: "Can I complete this course on a Mac?"
- User needs lab setup planning (educators)
- User wants to know if existing hardware is sufficient

**Example Usage**:

```bash
# In Claude Code, reference this agent:
@hardware-advisor I have $500 budget and a Mac, what hardware should I get?
```

**Example Interactions**:

**Input**: "I have $500 budget and want to learn robotics. What should I buy?"
**Output**: "With a $500 budget, I recommend starting with free tools and investing in a Jetson Orin Nano kit if you can stretch slightly: **Option 1: $0** - Use Docker Desktop (free), run Gazebo simulations, learn ROS 2 fundamentals. **Option 2: $697** - Jetson Orin Nano Super ($249), Intel RealSense D435i ($349), ReSpeaker Mic ($69), MicroSD ($30). This gives you real hardware for edge AI deployment!"

**Input**: "I have a Mac. Can I complete this course?"
**Output**: "Yes! You can complete this course on a Mac with workarounds: **Docker + Cloud approach** - Use Docker for ROS 2 + Gazebo (Modules 1-2), use Omniverse Cloud or AWS g5.2xlarge for Isaac Sim (Modules 3-4). Total cost: ~$180/quarter for cloud GPU or $0 if you skip Isaac Sim and focus on Gazebo."

**Input**: "I already have an RTX 3080 with 10GB VRAM. Do I need to upgrade?"
**Output**: "Your RTX 3080 (10GB VRAM) will work for most of the course! You can run ROS 2, Gazebo, and basic Isaac Sim scenes. For complex simulations, use cloud GPU (~$15 total for advanced exercises). Don't upgrade - save your money or invest in Jetson Orin Nano ($249) to learn edge deployment instead."

---

## Creating New Agents

To create a new custom agent for this project:

1. **Create Agent Configuration** (`.claude/agents/your-agent.json`):

```json
{
  "name": "your-agent-name",
  "description": "Brief description of what this agent does",
  "version": "1.0.0",
  "author": "Your Name",
  "capabilities": [
    "Capability 1",
    "Capability 2"
  ],
  "tools": [
    "read",
    "grep",
    "bash"
  ],
  "systemPrompt": "You are an expert at...",
  "examples": [
    {
      "input": "Example question",
      "output": "Example response"
    }
  ],
  "tags": ["tag1", "tag2"]
}
```

2. **Define Clear Capabilities**: Be specific about what the agent can and cannot do

3. **Provide Examples**: Include 2-3 real-world examples of how the agent should respond

4. **Test the Agent**: Invoke it with sample queries to verify behavior

---

## Agent Best Practices

### 1. **Specificity**
- Agents should be highly specialized for specific tasks
- Avoid creating "general-purpose" agents - they're less effective

### 2. **Tool Selection**
- Only include tools the agent actually needs
- More tools ≠ better agent (can cause confusion)

### 3. **Clear System Prompts**
- Define the agent's role, knowledge boundaries, and response format
- Include examples of desired behavior

### 4. **Hardware Awareness**
- For this project, agents should understand hardware differences (GPU vs Jetson vs Cloud)
- Always provide hardware-specific recommendations when relevant

### 5. **Documentation**
- Document when to use each agent
- Provide example invocations
- Keep examples up-to-date with book content

---

## Integration with RAG System

The `book-rag-helper` agent complements the RAG chatbot:

| Feature | RAG Chatbot | book-rag-helper Agent |
|---------|-------------|----------------------|
| **Use Case** | User-facing Q&A on the website | Development/debugging assistance |
| **Context** | Qdrant vector search + user profile | Direct file access + grep |
| **Speed** | Slower (embedding + LLM) | Faster (direct file reads) |
| **Accuracy** | Semantic search (can miss exact matches) | Precise (grep for exact terms) |
| **Best For** | General questions, conversational queries | Finding specific code, debugging, development |

**When to Use Each**:
- **RAG Chatbot**: User asks "What is ROS 2?" → Semantic understanding
- **book-rag-helper**: Developer asks "Where is the URDF example?" → Exact file location

---

## Troubleshooting

### Agent Not Responding
- Verify JSON syntax in agent configuration
- Check that all required fields are present (name, description, systemPrompt)
- Ensure tools are spelled correctly

### Agent Gives Wrong Answers
- Review and refine the `systemPrompt`
- Add more specific examples
- Reduce tool access if the agent is using incorrect tools

### Agent Too Slow
- Reduce tool usage (especially bash commands)
- Simplify the system prompt
- Use more specific queries to reduce search space

---

## Metrics & Success Criteria

Track agent effectiveness:
- **Response Accuracy**: Does the agent provide correct chapter/section references?
- **Hardware Awareness**: Does it adapt recommendations to user's hardware?
- **Response Time**: Agents should respond within 2-5 seconds
- **User Satisfaction**: Are developers finding agents helpful?

---

## Future Improvements

**Potential New Agents**:
1. `deployment-helper` - Assists with Vercel/GitHub Pages deployment
2. `ros2-debugger` - Helps debug ROS 2 node issues
3. `hardware-advisor` - Recommends hardware based on user budget
4. `content-translator` - Assists with Urdu translation verification

---

## Support & Contributions

**Questions?**
- Check the main [README.md](../../README.md)
- Review [Claude Code documentation](https://claude.com/claude-code)
- Open an issue in the GitHub repo

**Want to contribute a new agent?**
1. Follow the "Creating New Agents" guide above
2. Test thoroughly with real-world queries
3. Document examples and use cases
4. Submit a pull request with your agent JSON + documentation updates

---

**Last Updated**: 2025-12-23
**Maintainer**: Physical AI & Humanoid Robotics Team
**Version**: 1.0.0
