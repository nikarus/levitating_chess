---
description: Load always and keep in memory
---

We are working on levitating chess idea. Please read the PRODUCT_VISION.md.
You can also refer to the articles in the articles/ folder.

Read and follow the workspace organization rules in [methodology/README.md](methodology/README.md). Read other project material selectively for the task; do not load all documentation by default.

IMPORTANT: It is absolutely forbidden for AI to modify PRODUCT_VISION.md and PRODUCT_TESTING.md.
AI can only propose the modification to the user even when the user asks for it. The user is the only one who can modify these files.


Coding rules:
1. No comments in the code unless absolutely necessary or asked by user.
2. Variable and function names should be descriptive - it should be clear what they do.
3. No exception handling. No fallbacks of any kind. All code must work in the intended way. If it crashes - it should crash.
4. Always be vigelant not to duplicate functionality, not to introduce similar variables. Always keep the full context of the project in mind. This means: EVERY time you are adding something new, you should check if there is something similar already in the codebase. If there is - you should take this into account - reuse, or reuse partly removing parts that are not needed anymore, or remove it completely and add new.
5. When you run a long process, never monitor it periodically as it runs. You are expected to lock yourself against it and only resume and check results when it finishes.
