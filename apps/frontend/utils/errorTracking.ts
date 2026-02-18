type LogEntry = {
  timestamp: string;
  type: "log" | "warn" | "error" | "exception";
  message: string;
  context?: unknown;
};

class ErrorTracker {
  private logs: LogEntry[] = [];
  private maxLogs = 100;

  constructor() {
    if (typeof window !== "undefined") {
      this.initConsoleHooks();
    }
  }

  private initConsoleHooks() {
    const originalLog = console.log;
    const originalWarn = console.warn;
    const originalError = console.error;

    console.log = (...args: unknown[]) => {
      this.addLog("log", args);
      originalLog.apply(console, args);
    };

    console.warn = (...args: unknown[]) => {
      this.addLog("warn", args);
      originalWarn.apply(console, args);
    };

    console.error = (...args: unknown[]) => {
      this.addLog("error", args);
      originalError.apply(console, args);
    };
  }

  private addLog(type: LogEntry["type"], args: unknown[]) {
    const message = args
      .map((arg) => {
        if (typeof arg === "string") return arg;
        try {
          return JSON.stringify(arg);
        } catch {
          return String(arg);
        }
      })
      .join(" ");

    this.logs.push({
      timestamp: new Date().toISOString(),
      type,
      message,
    });

    if (this.logs.length > this.maxLogs) {
      this.logs.shift();
    }
  }

  captureException(error: Error, context: unknown = {}) {
    this.logs.push({
      timestamp: new Date().toISOString(),
      type: "exception",
      message: `${error.name}: ${error.message}\n${error.stack || ""}`,
      context,
    });

    if (this.logs.length > this.maxLogs) {
      this.logs.shift();
    }
  }

  getLogs() {
    return this.logs;
  }

  getErrors() {
    return this.logs.filter((l) => l.type === "error" || l.type === "exception");
  }

  clear() {
    this.logs = [];
  }
}

const errorTracker = new ErrorTracker();
export default errorTracker;

