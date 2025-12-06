---
sidebar_position: 3
title: ROS 2 Services
---

# ROS 2 Services: Synchronous AI Inference

Services provide request/response communication for AI inference calls.

## AI + Physical Integration

- **Physical robot requests AI inference**: "Is this object graspable?"
- **AI responds with decision**: "Yes, grasp at (x,y,z)"
- **Physical robot executes grasp**

### Example: LLM Planning Service

```python
# Service server: LLM generates pick-and-place plan
self.planning_service = self.create_service(
    PlanGrasp, 'plan_grasp', self.plan_callback
)

def plan_callback(self, request, response):
    # AI reasoning: Use LLM to plan grasp trajectory
    llm_response = openai_client.chat.completions.create(...)
    response.grasp_pose = parse_llm_output(llm_response)
    return response
```

**Physical Execution**: Robot calls service → LLM plans trajectory → Robot executes motion

## Key Concepts
- Service client/server model
- Custom service definitions
- Synchronous vs. asynchronous calls

See [ROS 2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Services.html).
