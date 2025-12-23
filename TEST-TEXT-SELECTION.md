# How to Test Text Selection Feature

## Quick Visual Test (1 minute)

### Step 1: Open Live Site
Visit: https://humanoidrobotbook.vercel.app/docs/introduction

### Step 2: Select Text
1. Use your mouse to **highlight any paragraph** (at least 5+ characters)
2. You should immediately see a **💬 Ask Me** button appear above the selection
3. The button has a chat icon and says "Ask Me"

**Expected Behavior**:
- Button appears within 100ms of selection
- Button is positioned above the highlighted text
- Button has gradient styling (purple/blue)

### Step 3: Click the Button
1. Click the **💬 Ask Me** button
2. Chatbot should **auto-open** in the bottom-right corner
3. Input field should show: `About this text: "your selected text..."`

**Expected Behavior**:
- Chatbot opens immediately
- Preview of your selection appears in input
- Ready to type question

### Step 4: Ask a Question
1. Type: "What does this mean?"
2. Press Enter
3. Wait for response (3-10 seconds)

### Step 5: Verify Backend Received Selection (Developer Check)

**Open Browser DevTools**:
1. Press **F12** (or Right-click → Inspect)
2. Go to **Network** tab
3. Filter by: `query`
4. Send your question
5. Click on the request to `/api/chat/query`
6. Go to **Payload** or **Request** tab

**Look for this in the request body**:
```json
{
  "question": "What does this mean?",
  "text_selection": {
    "text": "your highlighted text here...",
    "chapter_id": "selected-content",
    "start_offset": 0,
    "end_offset": 123
  }
}
```

**✅ SUCCESS**: If you see `text_selection` object in the request
**❌ FAILURE**: If `text_selection` is missing or null

---

## Mobile Testing (Optional)

### iOS Safari / Android Chrome:
1. Long-press on text to select
2. Drag selection handles
3. "Ask Me" button should appear
4. Tap button → Chatbot opens

**Note**: Mobile selection is slightly slower (touch handling)

---

## What You Should See

### Before Clicking "Ask Me":
```
┌─────────────────────────────────────┐
│  Physical AI & Humanoid Robotics    │
│                                     │
│  ROS 2 (Robot Operating System 2)  │  ← Selected Text
│  is a middleware framework...       │
│            💬 Ask Me  ← Button      │
│                                     │
└─────────────────────────────────────┘
```

### After Clicking:
```
┌─────────────────────────────────────┐
│  Physical AI & Humanoid Robotics    │
│                                     │
│  ROS 2 (Robot Operating System 2)  │
│  is a middleware framework...       │
│                                     │
│  ┌─────────────────────────────┐   │
│  │ Book Assistant            ✕ │   │
│  │                             │   │
│  │ About this text: "ROS 2..." │  ← Preview
│  │                             │   │
│  │ [Type your question...]     │   │
│  └─────────────────────────────┘   │
└─────────────────────────────────────┘
```

---

## Troubleshooting

### Button Not Appearing?
- Select **more than 5 characters**
- Try desktop browser first (mobile can be slower)
- Refresh the page and try again
- Check browser console (F12 → Console) for errors

### Button Appears But Chatbot Doesn't Open?
- Check browser console for JavaScript errors
- Try a different browser
- Clear cache and refresh

### Request Doesn't Include text_selection?
- This means my fix didn't deploy properly
- Let me know and I'll redeploy

---

## Expected Results

**Local Testing** (our curl test earlier):
```json
{
  "message_id": "2e421e78-7305-427e-8289-fd02a20aca2b",
  "response": "This Python code creates a ROS 2 node...",
  "metadata": {
    "latency_ms": 6532,
    "tokens_used": 669
  }
}
```

**Frontend Testing** (what you should see):
- Selection → Button → Chatbot → Response
- Full flow should take 5-15 seconds total
- Response should be relevant to selected text

---

## Quick Test Script (Copy-Paste This)

1. Visit: https://humanoidrobotbook.vercel.app/docs/introduction
2. Select this exact text: "ROS 2 (Robot Operating System 2)"
3. Click "Ask Me"
4. Type: "What is this?"
5. Press Enter
6. Check DevTools Network tab for `text_selection` in request

**Expected Response**: Chatbot should explain ROS 2 based on your selection

---

## Files to Check (If Button Doesn't Appear)

**Frontend Component**:
- `humanoid_robot_book/src/components/TextSelection/TextSelection.tsx`
- Should be integrated in Root.tsx

**If you want to debug**:
1. Open browser console (F12)
2. Type: `window.getSelection().toString()`
3. Should show your selected text

---

**Need Help?** Let me know what you see at each step!
