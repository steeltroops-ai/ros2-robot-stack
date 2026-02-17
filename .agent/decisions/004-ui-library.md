# Architectural Decision: UI Component Library (shadcn/ui)

## The Problem

Manually styling every button, card, and input (even with Tailwind) is slow and leads to inconsistency.
We need a **Production-Grade Design System** that:

1.  Is **Accessible** (screen readers, keyboard nav).
2.  Is **Customizable** (we own the code, not an npm package).
3.  Looks **Enterprise/Professional** out of the box.

## The Solution: shadcn/ui

We will use **shadcn/ui** (built on Radix Primitives + Tailwind).

### Why shadcn/ui?

- **Not a Library**: It is a collection of reusable components that you copy/paste (via CLI) into your project.
- **Enterprise Standard**: Used by Vercel, OpenAI, and major startups.
- **Headless**: Powered by Radix UI, ensuring WAI-ARIA compliance for complex widgets (Dialogs, Dropdowns, Tabs).

## Implementation Strategy

1.  **Initialize**: `npx shadcn-ui@latest init` (in `apps/frontend`).
2.  **Theme**: Configure to match our `globals.css` (Slate/Blue).
3.  **Components to Install**:
    - `Button`
    - `Card` (for Dashboard widgets)
    - `Badge` (for Status indicators)
    - `Table` (for Fleet lists)
    - `Sheet` (for Mobile Sidebar)

## Alternatives Rejected

- **Material UI (MUI)**: Too heavy, runtime CSS-in-JS performance cost.
- **Ant Design**: Difficult to customize, huge bundle size.
- **Chakra UI**: Runtime style props add overhead.

**Verdict**: `shadcn/ui` + Tailwind is the modern gold standard for high-performance React apps.
