"use client";

import {
  createContext,
  useContext,
  useState,
  useCallback,
  type ReactNode,
} from "react";

// ─── Types ───────────────────────────────────────────────────────────────────

interface RightPanelState {
  /** The JSX to render inside the panel body */
  content: ReactNode;
  /** Short label shown in the panel header (e.g. "Control Station") */
  title: string;
  /** Optional badge text next to the title (e.g. "LIVE", "SIM") */
  badge?: string;
  /** Badge colour variant */
  badgeVariant?: "green" | "amber" | "blue" | "grey";
}

interface RightPanelContextValue extends RightPanelState {
  setPanel: (state: Partial<RightPanelState>) => void;
  clearPanel: () => void;
}

// ─── Defaults ────────────────────────────────────────────────────────────────

const DEFAULT_STATE: RightPanelState = {
  content: null,
  title: "Control Station",
  badge: undefined,
  badgeVariant: "grey",
};

// ─── Context ─────────────────────────────────────────────────────────────────

const RightPanelContext = createContext<RightPanelContextValue>({
  ...DEFAULT_STATE,
  setPanel: () => {},
  clearPanel: () => {},
});

// ─── Provider ────────────────────────────────────────────────────────────────

export function RightPanelProvider({ children }: { children: ReactNode }) {
  const [state, setState] = useState<RightPanelState>(DEFAULT_STATE);

  const setPanel = useCallback((partial: Partial<RightPanelState>) => {
    setState((prev) => ({ ...prev, ...partial }));
  }, []);

  const clearPanel = useCallback(() => {
    setState(DEFAULT_STATE);
  }, []);

  return (
    <RightPanelContext.Provider value={{ ...state, setPanel, clearPanel }}>
      {children}
    </RightPanelContext.Provider>
  );
}

// ─── Hook ────────────────────────────────────────────────────────────────────

export function useRightPanel() {
  return useContext(RightPanelContext);
}
