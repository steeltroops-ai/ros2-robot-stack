"use client";

import DashboardLayout from "@/components/layout/DashboardLayout";
import { useFleetTelemetry } from "@/hooks/useFleetTelemetry";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "@/components/ui/table";

export default function Home() {
  const { robots, isConnected } = useFleetTelemetry();

  return (
    <DashboardLayout>
      <div className="space-y-6">
        {/* Header */}
        <div className="flex items-center justify-between">
          <h1 className="text-2xl font-bold tracking-tight text-[rgb(var(--text-primary))]">
            Fleet Overview
          </h1>
          <div className="flex items-center space-x-2">
            <Badge variant={isConnected ? "default" : "destructive"}>
              {isConnected ? "System Online" : "Disconnected"}
            </Badge>
          </div>
        </div>

        {/* Updated KPI Grid */}
        <div className="grid grid-cols-1 md:grid-cols-4 gap-6">
          <Card>
            <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
              <CardTitle className="text-sm font-medium">
                Active Units
              </CardTitle>
            </CardHeader>
            <CardContent>
              <div className="text-2xl font-bold">{robots.length}</div>
              <p className="text-xs text-muted-foreground">
                +1 since last hour
              </p>
            </CardContent>
          </Card>

          <Card>
            <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
              <CardTitle className="text-sm font-medium">Avg Battery</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="text-2xl font-bold">
                {robots.length > 0
                  ? Math.round(
                      robots.reduce((acc, r) => acc + r.battery, 0) /
                        robots.length,
                    )
                  : 0}
                %
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Live Fleet Table */}
        <div className="border rounded-lg overflow-hidden">
          <Table>
            <TableHeader>
              <TableRow>
                <TableHead>Robot ID</TableHead>
                <TableHead>Status</TableHead>
                <TableHead>Battery</TableHead>
                <TableHead>Position (X, Y)</TableHead>
                <TableHead className="text-right">Heading</TableHead>
              </TableRow>
            </TableHeader>
            <TableBody>
              {robots.map((robot) => (
                <TableRow key={robot.id}>
                  <TableCell className="font-medium">{robot.id}</TableCell>
                  <TableCell>
                    <Badge
                      variant="outline"
                      className="bg-emerald-50 text-emerald-700 border-emerald-200"
                    >
                      Active
                    </Badge>
                  </TableCell>
                  <TableCell>
                    <div className="flex items-center">
                      <div className="w-16 h-2 bg-gray-200 rounded-full mr-2 overflow-hidden">
                        <div
                          className={`h-full ${robot.battery > 20 ? "bg-emerald-500" : "bg-red-500"}`}
                          style={{ width: `${robot.battery}%` }}
                        />
                      </div>
                      <span className="text-xs font-mono">
                        {robot.battery.toFixed(1)}%
                      </span>
                    </div>
                  </TableCell>
                  <TableCell className="font-mono text-xs">
                    [{robot.x.toFixed(2)}, {robot.y.toFixed(2)}]
                  </TableCell>
                  <TableCell className="text-right font-mono text-xs">
                    {(robot.theta * (180 / Math.PI)).toFixed(1)}°
                  </TableCell>
                </TableRow>
              ))}
              {robots.length === 0 && (
                <TableRow>
                  <TableCell
                    colSpan={5}
                    className="text-center text-muted-foreground py-8"
                  >
                    No robots detected. Start simulation to see data.
                  </TableCell>
                </TableRow>
              )}
            </TableBody>
          </Table>
        </div>
      </div>
    </DashboardLayout>
  );
}
