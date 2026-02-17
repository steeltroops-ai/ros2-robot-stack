import Sidebar from "./Sidebar";

export default function DashboardLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <div className="flex h-screen w-full bg-[rgb(var(--bg-secondary))] text-[rgb(var(--text-primary))]">
      {/* Fixed Sidebar */}
      <aside className="fixed inset-y-0 left-0 z-50 w-64">
        <Sidebar />
      </aside>

      {/* Main Content Area */}
      <main className="pl-64 flex-1 h-full overflow-y-auto">
        <div className="max-w-7xl mx-auto p-8">{children}</div>
      </main>
    </div>
  );
}
