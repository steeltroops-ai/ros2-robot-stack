"use client";

import React, { Component, ReactNode, ErrorInfo } from "react";
import { motion, AnimatePresence } from "framer-motion";
import { FiChevronDown, FiChevronUp } from "react-icons/fi";
import {
  RiRefreshLine,
  RiHome5Line,
  RiFileCopyLine,
  RiCheckLine,
} from "react-icons/ri";
import errorTracker from "@/utils/errorTracking";

interface Props {
  children: ReactNode;
}

interface State {
  hasError: boolean;
  error: Error | null;
  errorInfo: ErrorInfo | null;
  showDetails: boolean;
  copied: boolean;
  recoveryCount: number;
  errorCategory: string | null;
  errorCode: string | null;
  errorMessage: string | null;
}

class ErrorBoundary extends Component<Props, State> {
  // Error Classification System
  static ERROR_CATEGORIES: Record<string, { color: string; label: string }> = {
    HTTP_CLIENT: { color: "orange", label: "Client Error" },
    HTTP_SERVER: { color: "red", label: "Server Error" },
    RUNTIME: { color: "purple", label: "Runtime Error" },
    NETWORK: { color: "yellow", label: "Network Error" },
    AUTH: { color: "cyan", label: "Authentication Error" },
    DATA: { color: "pink", label: "Data Error" },
    UNKNOWN: { color: "neutral", label: "Unknown Error" },
  };

  static HTTP_STATUS_MESSAGES: Record<number, string> = {
    400: "Bad Request - Invalid data sent to server",
    401: "Unauthorized - Authentication required",
    403: "Forbidden - Access denied to this resource",
    404: "Not Found - The requested resource does not exist",
    500: "Internal Server Error - Something went wrong on our end",
    502: "Bad Gateway - Server communication failed",
    503: "Service Unavailable - Server is temporarily down",
  };

  constructor(props: Props) {
    super(props);
    this.state = {
      hasError: false,
      error: null,
      errorInfo: null,
      showDetails: false,
      copied: false,
      recoveryCount: 0,
      errorCategory: null,
      errorCode: null,
      errorMessage: null,
    };
  }

  componentDidMount() {
    // Catch window errors (async, event handlers, etc.)
    window.addEventListener("error", this.handleGlobalError);
    // Catch unhandled promise rejections (async/await failures)
    window.addEventListener("unhandledrejection", this.handlePromiseError);
  }

  componentWillUnmount() {
    window.removeEventListener("error", this.handleGlobalError);
    window.removeEventListener("unhandledrejection", this.handlePromiseError);
  }

  handleGlobalError = (event: ErrorEvent) => {
    // Prevent double-reporting if componentDidCatch already got it
    if (this.state.hasError) return;

    this.processError(event.error || new Error(event.message));
  };

  handlePromiseError = (event: PromiseRejectionEvent) => {
    if (this.state.hasError) return;

    const error = event.reason instanceof Error ? event.reason : new Error(String(event.reason));
    this.processError(error);
  };

  processError(error: Error) {
    const { code, category, message } = this.classifyError(error);

    errorTracker.captureException(error, {
      isFatal: true,
      errorCode: code,
      errorCategory: category,
      source: "global_listener"
    });

    this.setState({
      hasError: true,
      error,
      errorCode: code,
      errorCategory: category,
      errorMessage: message,
    });
  }

  // Classify and extract error information
  classifyError(error: any) {
    let code = null;
    let category = "UNKNOWN";
    let message = "An unexpected error occurred";

    // Check for HTTP status codes
    if (error.status || error.code) {
      code = error.status || error.code;
      const codeNum = parseInt(code);

      if (codeNum >= 400 && codeNum < 500) {
        category = "HTTP_CLIENT";
        if (codeNum === 401) category = "AUTH";
      } else if (codeNum >= 500) {
        category = "HTTP_SERVER";
      }

      message =
        ErrorBoundary.HTTP_STATUS_MESSAGES[codeNum] || `HTTP Error ${code}`;
    }
    // Check for JavaScript runtime errors
    else if (error.name) {
      switch (error.name) {
        case "ReferenceError":
          code = "REF_ERR";
          category = "RUNTIME";
          message = "Variable or function not found";
          break;
        case "TypeError":
          code = "TYPE_ERR";
          category = "RUNTIME";
          message = "Invalid data type or operation";
          break;
        case "SyntaxError":
          code = "SYNTAX_ERR";
          category = "RUNTIME";
          message = "Code syntax error detected";
          break;
        case "RangeError":
          code = "RANGE_ERR";
          category = "RUNTIME";
          message = "Value out of valid range";
          break;
        default:
          if (error.name === "Error") {
            code = "RUNTIME";
            category = "RUNTIME";
            message = error.message || "Runtime exception occurred";
          } else {
            const cleanName = error.name.replace(/Error$/i, "");
            code = cleanName ? cleanName.toUpperCase() : "RUNTIME";
            category = "RUNTIME";
            message = error.message || "Runtime exception occurred";
          }
      }
    }
    else if (error.message) {
      if (error.message.includes("hook")) {
        code = "HOOK_ERR";
        category = "RUNTIME";
        message = "React Hook rules violated";
      } else if (error.message.includes("render")) {
        code = "RENDER_ERR";
        category = "RUNTIME";
        message = "Component rendering failed";
      } else if (
        error.message.includes("network") ||
        error.message.includes("fetch")
      ) {
        code = "NET_ERR";
        category = "NETWORK";
        message = "Network connection failed";
      } else if (
        error.message.includes("auth") ||
        error.message.includes("token")
      ) {
        code = "AUTH_ERR";
        category = "AUTH";
        message = "Authentication failed";
      } else if (
        error.message.includes("JSON") ||
        error.message.includes("parse")
      ) {
        code = "DATA_ERR";
        category = "DATA";
        message = "Invalid data format received";
      } else {
        code = "RUNTIME_ERR";
        category = "RUNTIME";
        message = error.message;
      }
    }

    return { code: String(code), category, message };
  }

  static getDerivedStateFromError(error: Error) {
    return {
      hasError: true,
      error: error,
    };
  }

  componentDidCatch(error: Error, errorInfo: ErrorInfo) {
    if (this.state.hasError && this.state.error === error) return;
    
    console.error("Critical System Breach:", error, errorInfo);

    const { code, category, message } = this.classifyError(error);

    errorTracker.captureException(error, {
      componentStack: errorInfo.componentStack,
      isFatal: true,
      errorCode: code,
      errorCategory: category,
      source: "component_did_catch"
    });

    this.setState({
      hasError: true,
      error,
      errorInfo,
      errorCode: code,
      errorCategory: category,
      errorMessage: message,
    });
  }

  resetError = () => {
    this.setState({
      hasError: false,
      error: null,
      errorInfo: null,
      showDetails: false,
    });
  };

  handleRefresh = () => {
    if (this.state.recoveryCount === 0) {
      this.setState((prev) => ({ recoveryCount: prev.recoveryCount + 1 }));
      this.resetError();
    } else {
      window.location.reload();
    }
  };

  handleCopy = async () => {
    const errorMsg = this.state.error?.message || String(this.state.error);
    const stack =
      this.state.errorInfo?.componentStack || "No stack trace available.";
    const report = `SYSTEM ERROR REPORT\n${"-".repeat(20)}\nError: ${errorMsg}\n\nStack:\n${stack}`;

    try {
      await navigator.clipboard.writeText(report);
      this.setState({ copied: true });
      setTimeout(() => this.setState({ copied: false }), 2000);
    } catch (err) {
      console.warn("Clipboard access denied:", err);
    }
  };

  render() {
    if (this.state.hasError) {
      return (
        <div
          role="alert"
          aria-live="assertive"
          className="overflow-x-hidden antialiased text-zinc-900 selection:bg-zinc-200 selection:text-black min-h-screen relative font-sans"
        >
          {/* Background - Bento Light Mesh */}
          <div className="fixed top-0 w-full h-full -z-10">
            <div className="relative w-full h-full bg-[#f5f5f7]">
              <div className="absolute bottom-0 left-0 right-0 top-0 bg-[linear-gradient(to_right,#00000012_1px,transparent_1px),linear-gradient(to_bottom,#00000012_1px,transparent_1px)] bg-[size:14px_24px]"></div>
              <div className="absolute left-0 right-0 top-[-10%] h-[1000px] w-[1000px] rounded-full bg-[radial-gradient(circle_400px_at_50%_300px,#ffffff99,#f5f5f7)]"></div>
            </div>
          </div>

          {/* Content */}
          <div className="container px-4 sm:px-6 lg:px-8 mx-auto max-w-7xl min-h-screen flex flex-col items-center justify-center py-12 sm:py-20">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.6 }}
              className="text-center space-y-8 sm:space-y-10 w-full max-w-2xl"
            >
              {/* Error Header */}
              <div className="space-y-3 sm:space-y-4">
                <motion.h1
                  initial={{ scale: 0.5, opacity: 0 }}
                  animate={{ scale: 1, opacity: 1 }}
                  transition={{ delay: 0.1, type: "spring", stiffness: 100 }}
                  className="text-6xl sm:text-7xl md:text-8xl font-black text-black leading-none tracking-tighter"
                >
                  ERROR
                </motion.h1>
                {this.state.errorCode && (
                  <motion.div
                    initial={{ opacity: 0, y: -10 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ delay: 0.2 }}
                    className="text-3xl sm:text-4xl md:text-5xl font-mono font-black text-black/85 tracking-[0.15em] sm:tracking-[0.2em]"
                  >
                    {this.state.errorCode}
                  </motion.div>
                )}
              </div>

              {/* Message */}
              <motion.div
                initial={{ opacity: 0 }}
                animate={{ opacity: 1 }}
                transition={{ delay: 0.3 }}
                className="space-y-3 sm:space-y-4 px-4"
              >
                <h2 className="text-xl sm:text-2xl md:text-3xl font-bold text-zinc-900 leading-tight tracking-tight">
                  {this.state.errorMessage || "Something went wrong"}
                </h2>
                <p className="text-sm sm:text-base text-zinc-500 max-w-md sm:max-w-lg mx-auto leading-relaxed font-medium">
                  {this.state.errorCategory === "HTTP_SERVER"
                    ? "Our servers encountered an issue. Please try again in a moment."
                    : this.state.errorCategory === "NETWORK"
                      ? "Unable to connect to the server. Please check your internet connection."
                      : this.state.errorCategory === "AUTH"
                        ? "Your session may have expired. Please try logging in again."
                        : "The application encountered an unexpected error. We've been notified and are looking into it."}
                </p>
              </motion.div>

              {/* Action Buttons */}
              <motion.div
                initial={{ opacity: 0, y: 10 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.5 }}
                className="flex flex-col sm:flex-row gap-3 sm:gap-4 justify-center items-center pt-4 w-full"
              >
                <button
                  onClick={this.handleRefresh}
                  className="group relative inline-flex items-center justify-center gap-2.5 w-full max-w-xs sm:w-52 py-3.5 rounded-full bg-black text-white hover:opacity-90 transition-all duration-300 shadow-sm overflow-hidden"
                >
                  <RiRefreshLine className="w-4 h-4 group-hover:rotate-180 transition-transform duration-700 ease-in-out flex-shrink-0" />
                  <span className="text-sm font-bold tracking-wide whitespace-nowrap">
                    {this.state.recoveryCount > 0
                      ? "Hard System Reset"
                      : "Perform Recovery"}
                  </span>
                </button>
                <button
                  onClick={() => (window.location.href = "/")}
                  className="group inline-flex items-center justify-center gap-2.5 w-full max-w-xs sm:w-52 py-3.5 rounded-full bg-white text-zinc-900 border border-zinc-200 hover:bg-zinc-50 transition-all duration-300 shadow-sm"
                >
                  <RiHome5Line className="w-4 h-4 group-hover:-translate-y-0.5 transition-transform duration-300 flex-shrink-0" />
                  <span className="text-sm font-bold tracking-wide whitespace-nowrap">
                    Return to Base
                  </span>
                </button>
              </motion.div>

              {/* Error Details */}
              {this.state.error && (
                <motion.div
                  initial={{ opacity: 0 }}
                  animate={{ opacity: 1 }}
                  transition={{ delay: 0.7 }}
                  className="pt-8"
                >
                  <button
                    onClick={() =>
                      this.setState({ showDetails: !this.state.showDetails })
                    }
                    className="group inline-flex items-center gap-2 text-[0.625rem] font-bold text-zinc-700 hover:text-black transition-colors uppercase tracking-widest"
                  >
                    {this.state.showDetails ? (
                      <FiChevronUp />
                    ) : (
                      <FiChevronDown />
                    )}
                    {this.state.showDetails ? "Hide" : "Show"} Forensic Details
                  </button>

                  <AnimatePresence>
                    {this.state.showDetails && (
                      <motion.div
                        initial={{ opacity: 0, height: 0 }}
                        animate={{ opacity: 1, height: "auto" }}
                        exit={{ opacity: 0, height: 0 }}
                        className="mt-6 text-left px-2 sm:px-0"
                      >
                        <div className="relative rounded-[1.25rem] bg-white border border-zinc-300 shadow-2xl shadow-black/5 overflow-hidden group">
                          {/* Forensic Header */}
                          <div className="sticky top-0 z-30 flex items-center justify-between gap-2 py-3 px-4 sm:px-6 bg-zinc-50 border-b border-zinc-300">
                            <div className="flex items-center gap-2 sm:gap-3 overflow-hidden min-w-0">
                              <div className="flex gap-1.5 shrink-0">
                                <div className="w-3 h-3 rounded-full bg-[#FF5F56] border border-black/5" />
                                <div className="w-3 h-3 rounded-full bg-[#FFBD2E] border border-black/5" />
                                <div className="w-3 h-3 rounded-full bg-[#27C93F] border border-black/5" />
                              </div>
                              <span className="text-[10px] font-mono text-zinc-900 font-bold truncate ml-2 uppercase tracking-tight">
                                {this.state.error.message ||
                                  this.state.error.toString()}
                              </span>
                            </div>
                            <button
                              onClick={this.handleCopy}
                              className="flex items-center gap-1.5 px-3 py-1.5 rounded-lg bg-white border border-zinc-300 text-zinc-700 hover:text-black hover:border-black transition-all duration-200 shrink-0"
                            >
                              {this.state.copied ? (
                                <>
                                  <RiCheckLine className="w-3.5 h-3.5 text-green-500" />
                                  <span className="text-[10px] font-bold uppercase tracking-wider hidden sm:inline">
                                    Copied
                                  </span>
                                </>
                              ) : (
                                <>
                                  <RiFileCopyLine className="w-3.5 h-3.5" />
                                  <span className="text-[10px] font-bold uppercase tracking-wider hidden sm:inline">
                                    Copy Report
                                  </span>
                                </>
                              )}
                            </button>
                          </div>

                          {/* Forensic Body */}
                          <div className="relative max-h-[45vh] overflow-y-auto overflow-x-hidden scrollbar-hide">
                            <div className="p-4 sm:p-8 pt-4 sm:pt-6 space-y-4 sm:space-y-6">
                              <div className="space-y-4">
                                <div className="space-y-2">
                                  <span className="text-[10px] uppercase tracking-widest text-zinc-500 font-bold">
                                    Stack Trace
                                  </span>
                                  <div className="font-mono text-[10px] sm:text-xs text-black whitespace-pre-wrap break-words leading-relaxed bg-zinc-50 p-4 sm:p-6 rounded-[1rem] border border-zinc-300 overflow-hidden shadow-inner font-bold">
                                    {this.state.error?.stack || this.state.error?.toString() || "No stack trace available."}
                                  </div>
                                </div>

                                {this.state.errorInfo?.componentStack && (
                                  <div className="space-y-2">
                                    <span className="text-[10px] uppercase tracking-widest text-zinc-500 font-bold">
                                      Component Stack
                                    </span>
                                    <div className="font-mono text-[10px] sm:text-xs text-zinc-600 whitespace-pre-wrap break-words leading-relaxed bg-zinc-50/50 p-4 sm:p-6 rounded-[1rem] border border-zinc-200 overflow-hidden font-medium">
                                      {this.state.errorInfo.componentStack}
                                    </div>
                                  </div>
                                )}
                              </div>

                              {/* Console Logs / Bugs */}
                              <div className="space-y-3 pt-6">
                                <div className="flex items-center justify-between border-b border-zinc-300 pb-2">
                                  <span className="text-[10px] uppercase tracking-widest text-zinc-700 font-black">
                                    Console Intelligence & Known Bugs
                                  </span>
                                  <span className="text-[9px] font-mono text-zinc-400 font-bold">
                                    LAST {errorTracker.getLogs().length} EVENTS
                                  </span>
                                </div>
                                <div className="space-y-1.5 max-h-[300px] overflow-y-auto pr-2 scrollbar-hide">
                                  {errorTracker.getLogs().length > 0 ? (
                                    errorTracker.getLogs().map((log, idx) => (
                                      <div
                                        key={idx}
                                        className={`flex gap-3 text-[9px] sm:text-[10px] font-mono leading-tight border-l-2 pl-3 py-1.5 transition-colors ${
                                          log.type === "error" || log.type === "exception"
                                            ? "border-red-500 bg-red-50/30"
                                            : log.type === "warn"
                                            ? "border-yellow-500 bg-yellow-50/30"
                                            : "border-zinc-300 hover:bg-zinc-50"
                                        }`}
                                      >
                                        <span className="text-zinc-400 font-bold shrink-0 opacity-70">
                                          {new Date(log.timestamp).toLocaleTimeString([], {
                                            hour12: false,
                                            hour: '2-digit',
                                            minute: '2-digit',
                                            second: '2-digit'
                                          })}
                                        </span>
                                        <span className={`font-black shrink-0 uppercase tracking-tighter w-12 ${
                                          log.type === "error" || log.type === "exception"
                                            ? "text-red-600"
                                            : log.type === "warn"
                                            ? "text-yellow-600"
                                            : "text-zinc-500"
                                        }`}>
                                          {log.type}
                                        </span>
                                        <span className={`font-bold break-all ${
                                          log.type === "error" || log.type === "exception"
                                            ? "text-red-900"
                                            : log.type === "warn"
                                            ? "text-yellow-900"
                                            : "text-zinc-800"
                                        }`}>
                                          {log.message}
                                        </span>
                                      </div>
                                    ))
                                  ) : (
                                    <div className="text-[10px] font-mono text-zinc-400 italic py-4 text-center">
                                      No tactical logs recorded.
                                    </div>
                                  )}
                                </div>
                              </div>

                                {/* System Diagnostics */}
                                <div className="pt-6 border-t border-zinc-200 space-y-3">
                                  <div className="flex items-center gap-2">
                                    <span className="text-[10px] uppercase tracking-widest text-zinc-700 font-black">
                                      System Diagnostics
                                    </span>
                                  </div>
                                  <div className="grid grid-cols-2 sm:grid-cols-3 gap-2">
                                    <div className="bg-zinc-50 border border-zinc-200 p-3 rounded-xl space-y-1">
                                      <div className="text-[8px] uppercase tracking-tighter text-zinc-400 font-bold">Memory Usage</div>
                                      <div className="text-[10px] font-mono font-black text-black">
                                        {(performance as any).memory ? `${Math.round((performance as any).memory.usedJSHeapSize / 1048576)}MB` : "UNAVAILABLE"}
                                      </div>
                                    </div>
                                    <div className="bg-zinc-50 border border-zinc-200 p-3 rounded-xl space-y-1">
                                      <div className="text-[8px] uppercase tracking-tighter text-zinc-400 font-bold">Latency</div>
                                      <div className="text-[10px] font-mono font-black text-black">
                                        {performance.now().toFixed(0)}ms
                                      </div>
                                    </div>
                                    <div className="bg-zinc-50 border border-zinc-200 p-3 rounded-xl space-y-1">
                                      <div className="text-[8px] uppercase tracking-tighter text-zinc-400 font-bold">Path</div>
                                      <div className="text-[10px] font-mono font-black text-black truncate italic">
                                        {window.location.pathname}
                                      </div>
                                    </div>
                                  </div>
                                </div>

                                {/* Debug Context */}
                                <div className="pt-8 border-t border-zinc-100 space-y-2 opacity-50">
                                  <div className="flex justify-between text-[9px] font-mono uppercase tracking-widest text-zinc-400 font-bold">
                                    <span>System Identity</span>
                                    <span>{new Date().toISOString()}</span>
                                  </div>
                                  <div className="text-[9px] font-mono text-zinc-500 truncate font-medium uppercase tracking-tighter">
                                    UA: {window.navigator.userAgent}
                                  </div>
                                </div>
                            </div>
                          </div>
                        </div>
                      </motion.div>
                    )}
                  </AnimatePresence>
                </motion.div>
              )}
            </motion.div>

            {/* Decorative Footer */}
            <motion.div
              initial={{ opacity: 0 }}
              animate={{ opacity: 0.8 }}
              transition={{ delay: 1 }}
              className="absolute bottom-4 sm:bottom-8 text-center text-[10px] uppercase font-black tracking-[0.3em] text-zinc-600 px-4"
            >
              Omniverse System Integrity Protocol • Stabilized
            </motion.div>
          </div>
        </div>
      );
    }

    return this.props.children;
  }
}

export default ErrorBoundary;
