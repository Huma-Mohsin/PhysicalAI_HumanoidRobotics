# Feature Specification: Text Selection RAG Queries

**Feature Branch**: `006-text-selection-rag`
**Created**: 2025-12-18
**Status**: Implementation (80% Complete)
**Priority**: P1 (Core Requirement - Constitution line 100)
**Effort**: Small (5-10 minutes remaining)
**Completion**: 80% - Frontend complete, backend needs 3-line fix

## Current Implementation Status

### ✅ Already Complete (Discovered 2025-12-18):

1. **Frontend TextSelection Component** (251 lines) - `humanoid_robot_book/src/components/TextSelection/TextSelection.tsx`
   - Fully implemented with mobile + desktop support
   - Shows "💬 Ask Me" button when text selected (>5 characters)
   - Handles selectionchange, mouseup, touchend events
   - Positions button above selection dynamically
   - Excellent mobile optimization (passive event listeners, touch support)

2. **Chatbot Integration** - `humanoid_robot_book/src/components/Chatbot/Chatbot.tsx:28-105`
   - Accepts `selectedText` prop from Root.tsx
   - Auto-opens chatbot when text is selected
   - Shows preview in input field
   - Sends `text_selection` object to backend API

3. **Backend TextSelection Model** - `backend/src/models/message.py:11-18`
   - Complete model with chapter_id, start_offset, end_offset, context
   - Database schema supports storing text_selection in messages

### ⚠️ Missing (5 minutes to fix):

**Backend API Endpoint** - `backend/src/api/chat.py:26-31`
- ChatQueryRequest missing `text_selection: Optional[TextSelection]` field
- Frontend sends data, backend silently ignores it
- Need to add 3 lines: import, field definition, pass to message creation

### 📝 Remaining Work:
1. Add text_selection to ChatQueryRequest (3 lines)
2. Test locally with curl
3. Deploy to Vercel production
4. Test with live frontend
5. Create PHR documentation

---

## User Scenarios & Testing

### User Story 1 - Highlight Text and Ask (Priority: P1) 🎯 MVP

As a reader studying a specific code example or paragraph, I want to highlight the text and ask a targeted question about it, so that the chatbot gives me context-specific answers instead of general book-wide responses.

**Why this priority**: This is explicitly required by constitution ("Text Selection Queries") and differentiates our RAG from generic chatbots.

**Independent Test**: Navigate to Module 1 → Highlight Python code snippet → Click "Ask about selection" button → Type "What does this do?" → Verify chatbot response references the highlighted code.

**Acceptance Scenarios**:

1. **Given** I am reading a chapter with code examples, **When** I highlight a code snippet with my mouse, **Then** a "Ask about this" button appears near the selection
2. **Given** I have highlighted text and clicked "Ask about this", **When** the chatbot opens with my selection pre-loaded, **Then** I see my highlighted text shown in the chat UI as context
3. **Given** I type a question about the highlighted text, **When** the chatbot responds, **Then** the answer specifically references my selection and provides targeted explanation

---

### User Story 2 - Selection Context Preservation (Priority: P2)

As a user who highlighted text from a specific chapter, I want the chatbot to know which chapter the selection came from, so that it can provide chapter-specific context in its answer.

**Why this priority**: Improves answer quality by giving chatbot full context (text + chapter + section).

**Independent Test**: Highlight text from Module 3 (Isaac Sim) → Ask "What hardware do I need?" → Verify chatbot mentions Isaac Sim requirements (not general ROS hardware).

**Acceptance Scenarios**:

1. **Given** I highlighted text from Module 3 (Isaac Sim) and asked a question, **When** the chatbot responds, **Then** the answer includes Isaac Sim-specific context (GPU requirements, Ubuntu version)
2. **Given** I highlighted text from Module 1 (ROS 2) and asked about installation, **When** the chatbot responds, **Then** the answer focuses on ROS 2 installation steps, not Isaac Sim

---

### Edge Cases

- What if user highlights across multiple paragraphs (>2000 characters)?
  - **Expected**: Truncate to 2000 chars, show warning "Selection too long, truncated"

- What if user highlights non-text content (images, diagrams)?
  - **Expected**: Button doesn't appear, or shows "Text selection only"

- What if user clicks "Ask about this" but then clears the selection?
  - **Expected**: Selection text still sent to chatbot (captured on button click)

- What if user highlights text then navigates to different chapter before asking?
  - **Expected**: Chapter context from original selection preserved in request

---

## Requirements

### Functional Requirements

- **FR-001**: System MUST detect text selection on chapter pages (mouse highlight)
- **FR-002**: System MUST show "Ask about this" button when text is selected (≥10 characters)
- **FR-003**: Clicking button MUST open chatbot with selection pre-loaded as context
- **FR-004**: Chatbot request MUST include: (a) selected text, (b) chapter_id, (c) start/end offsets, (d) user question
- **FR-005**: Backend MUST retrieve relevant context based on selection + question (hybrid search)
- **FR-006**: Backend response MUST reference the selected text in the answer
- **FR-007**: Selection context MUST be highlighted in chat UI (visual distinction from regular questions)
- **FR-008**: System MUST handle selections up to 2000 characters (truncate if longer)

### Key Entities

**TextSelection** (already exists in backend):
```typescript
{
  text: string;              // The highlighted text
  chapter_id: string;        // e.g., "03-module-1-ros2"
  start_offset: number;      // Character position in chapter
  end_offset: number;        // Character position in chapter
}
```

**ChatQueryRequest** (existing, already supports text_selection):
```typescript
{
  question: string;
  conversation_id?: string;
  session_id?: string;
  user_id?: string;
  text_selection?: TextSelection;  // ✅ Already in backend model
  top_k?: number;
  similarity_threshold?: number;
}
```

---

## Success Criteria

### Measurable Outcomes

- **SC-001**: "Ask about this" button appears within 100ms of text selection
- **SC-002**: Chatbot opens with selection context visible 100% of the time
- **SC-003**: Backend includes selection in RAG prompt for 100% of selection-based queries
- **SC-004**: Users can successfully ask selection-specific questions with ≤3 clicks

---

## Technical Implementation (High-Level)

### Frontend Changes (Small)

**File**: `humanoid_robot_book/src/components/TextSelection/TextSelection.tsx` (already exists!)

**Current Status**: Component exists but not integrated into chapter pages

**Changes Needed**:
1. Integrate TextSelection component into Docusaurus layout
2. Connect "Ask about this" button to Chatbot component
3. Pass `text_selection` object to chatbot state

**File**: `humanoid_robot_book/src/components/Chatbot/Chatbot.tsx`

**Changes Needed**:
1. Accept `initialTextSelection` prop
2. Display selection context in chat UI (highlighted box)
3. Include `text_selection` in API request to backend

### Backend Changes (Minimal)

**Status**: ✅ Backend already supports `text_selection` parameter!

**File**: `backend/src/api/chat.py` (line 44-48)

**Existing Code**:
```python
class ChatQueryRequest(BaseModel):
    question: str = Field(...)
    # ... other fields ...
    text_selection: Optional[TextSelection] = None  # ✅ Already exists!
```

**Enhancement Needed** (Optional):
- Weight selection text higher in RAG retrieval
- Include selection in prompt: "User highlighted: '{selection}'. Question: '{question}'"

### Data Flow

```
User highlights text on chapter page
  → TextSelection component captures:
     - text: "import rclpy\nfrom rclpy.node..."
     - chapter_id: "03-module-1-ros2"
     - start_offset: 1234
     - end_offset: 1456

User clicks "Ask about this"
  → Chatbot opens with selection shown

User types question: "What does this import do?"
  → Frontend sends to backend:
     {
       "question": "What does this import do?",
       "text_selection": {
         "text": "import rclpy...",
         "chapter_id": "03-module-1-ros2",
         ...
       }
     }

Backend RAG pipeline:
  → Retrieves context from Qdrant (hybrid: selection + question)
  → Generates answer with selection awareness
  → Returns response referencing the code

Frontend displays answer with selection highlighted
```

---

## Constitution Compliance

**Line 100**: ✅ "Text Selection Queries: Support user-highlighted text as query context"
- This feature directly satisfies this requirement

**Line 61**: ⚠️ Indirectly supports personalization (can combine with hardware profile in future)

---

## Dependencies

- ✅ Backend `TextSelection` model exists
- ✅ Backend API supports `text_selection` parameter
- ✅ Frontend `TextSelection` component exists (not integrated)
- ✅ Chatbot component functional

---

## Out of Scope (This Feature)

- Advanced text selection (images, tables, diagrams)
- Multi-chapter selection spanning
- Selection history/bookmarking
- Sharing selected text with others

---

## Risks & Mitigations

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Selection conflicts with Docusaurus | LOW | LOW | Test in production build, not just dev |
| Mobile selection UX poor | MEDIUM | MEDIUM | Use native mobile text selection UI |
| Backend doesn't prioritize selection | MEDIUM | MEDIUM | Enhance RAG prompt to emphasize selection |

---

## Implementation Effort

**Estimated Time**: 2-3 hours

**Breakdown**:
- Frontend integration: 1.5 hours
  - Connect TextSelection to layout
  - Wire to Chatbot component
  - UI polish (selection highlighting)
- Backend enhancement: 0.5 hours
  - Optional: Improve selection weighting in RAG
- Testing: 1 hour
  - Manual testing across chapters
  - Mobile testing

---

## Next Steps

1. **Planning**: Create simple plan in `specs/006-text-selection-rag/plan.md`
2. **Tasks**: Break down into 5-7 tasks in `specs/006-text-selection-rag/tasks.md`
3. **Implementation**: Integrate frontend components
4. **Testing**: Manual testing on 3+ chapters
5. **PHR**: Document in `history/prompts/006-text-selection-rag/`

---

**Approved**: Pending user confirmation
**Blocked By**: None (can start immediately)
**Blocks**: None (independent feature)
**Effort**: Small (mostly frontend integration)
