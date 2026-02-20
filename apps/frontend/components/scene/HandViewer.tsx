"use client";

import { Canvas } from "@react-three/fiber";
import { OrbitControls, PerspectiveCamera, Grid } from "@react-three/drei";
import * as THREE from "three";
import { BionicHand } from "./assets/BionicHand";

// ═══════════════════════════════════════════════════════════════════
// HandViewer — 3D Viewer for the Bionic Hand
// Research-lab aesthetic: warm off-white floor, soft daylight GI,
// detailed workbench/equipment, no cyberpunk colors.
// ═══════════════════════════════════════════════════════════════════

// ── Lab palette ─────────────────────────────────────────────────────
const LAB = {
  floor:       "#e4e1da",   // Warm off-white vinyl tile
  floorGrid:   "#c8c4bb",   // Subtle warm grid
  floorGridSec:"#b5b0a6",   // Section grid lines
  wall:        "#dcdad5",   // Warm off-white walls
  ceiling:     "#ede9e2",   // Warm ceiling
  // Furniture
  benchBody:   "#c9cdd6",   // Light grey metal bench frame
  benchTop:    "#a08060",   // Natural wood/laminate bench top
  benchLeg:    "#9aa0ac",   // Metal legs (brushed aluminium)
  cabinet:     "#d4d8de",   // White-grey lab cabinet
  cabinetDoor: "#c8ccd3",   // Slightly darker door panel
  rack:        "#b8bcc4",   // Equipment rack grey
  rackPanel:   "#3a4a5c",   // Dark instrument panel
  // Instruments & screens
  screenBody:  "#2a2e38",   // Monitor housing
  screenGlow:  "#3a8ab8",   // Monitor screen — lab blue
  ledGreen:    "#5ab85a",   // Power-on LED (muted green)
  ledAmber:    "#c8943a",   // Status LED amber
  // Metal details
  steel:       "#8090a0",   // Brushed steel
  darkSteel:   "#4a5260",   // Dark steel frame
  // Misc lab items
  trayOrange:  "#c87a40",   // Safety-orange parts tray
  cableGrey:   "#5a6070",   // Cable bundle
  whitePlastic:"#e0ddd8",   // White plastic enclosures
};

// ── Lighting ── warm lab fluorescent + natural fill ──────────────────
function SceneLighting() {
  return (
    <>
      {/* Hemisphere: warm white sky (ceiling) / warm grey ground (floor bounce) */}
      <hemisphereLight
        args={["#dce9f5", "#c4b89a", 0.9]}
      />

      {/* Primary overhead fluorescent bank */}
      <directionalLight
        position={[1, 9, 2]}
        intensity={2.4}
        castShadow
        shadow-mapSize-width={2048}
        shadow-mapSize-height={2048}
        shadow-bias={-0.0004}
        color="#f8f6f0"
      />

      {/* Left window fill — cool daylight from a window */}
      <directionalLight position={[-5, 4, 0]} intensity={1.0} color="#c8dcf0" />

      {/* Rear fill — soft bounce off back wall */}
      <directionalLight position={[0, 3, -6]} intensity={0.5} color="#ece8e0" />

      {/* Overhead work-light above the robot arm */}
      <pointLight
        position={[0.2, 2.2, 0.5]}
        intensity={3.0}
        color="#fff8f0"
        distance={5}
        decay={2}
      />
    </>
  );
}

// ═══ Torso / Shoulder Body ═══════════════════════════════════════════
function TorsoMount() {
  return (
    <group>
      {/* ── Base Mounting Plate ── */}
      <mesh position={[0, 0.015, 0]} castShadow receiveShadow>
        <cylinderGeometry args={[0.21, 0.23, 0.03, 32]} />
        <meshStandardMaterial color={LAB.darkSteel} roughness={0.4} metalness={0.7} />
      </mesh>
      {/* Base accent ring — polished aluminium */}
      <mesh position={[0, 0.031, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <ringGeometry args={[0.175, 0.188, 48]} />
        <meshStandardMaterial color={LAB.steel} roughness={0.2} metalness={0.9} />
      </mesh>

      {/* ── Spine / Support Column ── */}
      <mesh position={[0, 0.33, 0]} castShadow>
        <cylinderGeometry args={[0.03, 0.035, 0.90, 16]} />
        <meshStandardMaterial color="#4a5060" roughness={0.35} metalness={0.75} />
      </mesh>
      {/* Column ring detail */}
      <mesh position={[0, 0.22, 0]} castShadow>
        <cylinderGeometry args={[0.037, 0.037, 0.012, 16]} />
        <meshStandardMaterial color={LAB.steel} roughness={0.25} metalness={0.85} />
      </mesh>

      {/* ── Upper Torso / Chest Block ── */}
      <mesh position={[0, 0.70, 0]} castShadow receiveShadow>
        <boxGeometry args={[0.18, 0.28, 0.12]} />
        <meshStandardMaterial color="#2a2e35" roughness={0.55} metalness={0.45} />
      </mesh>
      {/* Torso front panel */}
      <mesh position={[0, 0.70, 0.061]}>
        <boxGeometry args={[0.14, 0.22, 0.003]} />
        <meshStandardMaterial color="#3a4048" roughness={0.35} metalness={0.55} />
      </mesh>
      {/* Chest amber status LED */}
      <mesh position={[0, 0.74, 0.063]}>
        <circleGeometry args={[0.015, 24]} />
        <meshStandardMaterial
          color={LAB.ledAmber}
          emissive={LAB.ledAmber}
          emissiveIntensity={0.9}
          toneMapped={false}
        />
      </mesh>
      {/* Chest panel lines */}
      {[0.62, 0.70, 0.78].map((y, i) => (
        <mesh key={`line-${i}`} position={[0, y, 0.063]}>
          <boxGeometry args={[0.12, 0.002, 0.001]} />
          <meshStandardMaterial color="#555d68" roughness={0.4} metalness={0.5} />
        </mesh>
      ))}

      {/* ── Neck ── */}
      <mesh position={[0, 0.860, 0]} castShadow>
        <cylinderGeometry args={[0.022, 0.028, 0.04, 12]} />
        <meshStandardMaterial color="#4a5060" roughness={0.4} metalness={0.6} />
      </mesh>

      {/* ── Head ── */}
      <group position={[0, 0.91, 0]} rotation={[0.3, 0, 0]}>
        <mesh castShadow>
          <sphereGeometry args={[0.045, 18, 14]} />
          <meshStandardMaterial color="#2e3340" roughness={0.45} metalness={0.45} />
        </mesh>
        {/* Visor strip — dark tinted glass */}
        <mesh position={[0, 0.005, 0.038]}>
          <boxGeometry args={[0.055, 0.016, 0.012]} />
          <meshStandardMaterial color="#1a2030" roughness={0.05} metalness={0.85} opacity={0.88} transparent />
        </mesh>
        {/* Sensor dots — muted amber */}
        <mesh position={[-0.014, 0.005, 0.044]}>
          <sphereGeometry args={[0.004, 8, 8]} />
          <meshStandardMaterial color={LAB.ledAmber} emissive={LAB.ledAmber} emissiveIntensity={0.9} toneMapped={false} />
        </mesh>
        <mesh position={[0.014, 0.005, 0.044]}>
          <sphereGeometry args={[0.004, 8, 8]} />
          <meshStandardMaterial color={LAB.ledAmber} emissive={LAB.ledAmber} emissiveIntensity={0.9} toneMapped={false} />
        </mesh>
        {/* Jaw */}
        <mesh position={[0, -0.030, 0.032]}>
          <boxGeometry args={[0.036, 0.010, 0.016]} />
          <meshStandardMaterial color="#3a4048" roughness={0.4} metalness={0.5} />
        </mesh>
      </group>

      {/* ── RIGHT SHOULDER SOCKET ── */}
      <mesh position={[0.10, 0.74, 0]} castShadow>
        <boxGeometry args={[0.04, 0.08, 0.08]} />
        <meshStandardMaterial color="#434a55" roughness={0.35} metalness={0.65} />
      </mesh>
      <mesh position={[0.13, 0.74, 0]} rotation={[0, 0, Math.PI / 2]} castShadow>
        <cylinderGeometry args={[0.042, 0.042, 0.025, 20]} />
        <meshStandardMaterial color="#505868" roughness={0.3} metalness={0.75} />
      </mesh>
      {/* Shoulder ring — polished steel */}
      <mesh position={[0.144, 0.74, 0]} rotation={[0, 0, Math.PI / 2]}>
        <ringGeometry args={[0.035, 0.040, 24]} />
        <meshStandardMaterial color={LAB.steel} roughness={0.2} metalness={0.9} />
      </mesh>

      {/* ── LEFT SHOULDER STUB ── */}
      <mesh position={[-0.10, 0.74, 0]} castShadow>
        <boxGeometry args={[0.04, 0.08, 0.08]} />
        <meshStandardMaterial color="#434a55" roughness={0.35} metalness={0.65} />
      </mesh>
      <mesh position={[-0.13, 0.74, 0]} rotation={[0, 0, Math.PI / 2]} castShadow>
        <cylinderGeometry args={[0.035, 0.035, 0.02, 20]} />
        <meshStandardMaterial color="#505868" roughness={0.3} metalness={0.75} />
      </mesh>
      <mesh position={[-0.141, 0.74, 0]} rotation={[0, 0, Math.PI / 2]}>
        <circleGeometry args={[0.034, 20]} />
        <meshStandardMaterial color="#3a4048" roughness={0.5} metalness={0.5} />
      </mesh>

      {/* ── Cable routing ── */}
      <mesh position={[0.06, 0.66, 0.03]} castShadow>
        <capsuleGeometry args={[0.004, 0.12, 4, 8]} />
        <meshStandardMaterial color={LAB.cableGrey} roughness={0.7} metalness={0.2} />
      </mesh>
      <mesh position={[0.07, 0.65, -0.02]} castShadow>
        <capsuleGeometry args={[0.003, 0.10, 4, 8]} />
        <meshStandardMaterial color={LAB.cableGrey} roughness={0.7} metalness={0.2} />
      </mesh>
    </group>
  );
}

// ═══ Lab Back Wall ═══════════════════════════════════════════════════
function LabWalls() {
  return (
    <group>
      {/* Back wall */}
      <mesh position={[0, 1.5, -2.2]} receiveShadow>
        <boxGeometry args={[7, 3.5, 0.06]} />
        <meshStandardMaterial color={LAB.wall} roughness={0.95} metalness={0.0} />
      </mesh>
      {/* Left side wall */}
      <mesh position={[-2.8, 1.5, 0]} receiveShadow>
        <boxGeometry args={[0.06, 3.5, 7]} />
        <meshStandardMaterial color={LAB.wall} roughness={0.95} metalness={0.0} />
      </mesh>
      {/* Ceiling */}
      <mesh position={[0, 3.0, 0]} receiveShadow>
        <boxGeometry args={[7, 0.05, 7]} />
        <meshStandardMaterial color={LAB.ceiling} roughness={0.95} metalness={0.0} />
      </mesh>

      {/* Ceiling fluorescent light fixtures */}
      {[[-0.5, 0], [1.2, 0]].map(([x, z], i) => (
        <group key={`fixture-${i}`} position={[x, 2.96, z as number]}>
          {/* Housing */}
          <mesh>
            <boxGeometry args={[0.08, 0.04, 0.7]} />
            <meshStandardMaterial color="#c8c5be" roughness={0.5} metalness={0.4} />
          </mesh>
          {/* Tube glow */}
          <mesh position={[0, -0.015, 0]}>
            <boxGeometry args={[0.018, 0.008, 0.62]} />
            <meshStandardMaterial
              color="#f0efeb"
              emissive="#d8d4c8"
              emissiveIntensity={1.2}
              toneMapped={false}
            />
          </mesh>
        </group>
      ))}

      {/* Wall baseboard / dado rail */}
      <mesh position={[0, 0.05, -2.17]} receiveShadow>
        <boxGeometry args={[7, 0.10, 0.04]} />
        <meshStandardMaterial color="#b8b4aa" roughness={0.8} metalness={0.15} />
      </mesh>
    </group>
  );
}

// ═══ Research Workbench ═══════════════════════════════════════════════
// A proper stainless-steel lab bench with 4 legs, a wood-laminate top,
// a lower shelf, and a back rail.
function Workbench({ position }: { position: [number, number, number] }) {
  const [bx, by, bz] = position;
  const W = 1.4, D = 0.6, H = 0.82; // width, depth, height
  const topThk = 0.04;
  const legR = 0.018;

  return (
    <group position={[bx, by, bz]}>
      {/* Bench top — wood laminate */}
      <mesh position={[0, H, 0]} castShadow receiveShadow>
        <boxGeometry args={[W, topThk, D]} />
        <meshStandardMaterial color={LAB.benchTop} roughness={0.75} metalness={0.0} />
      </mesh>
      {/* Top edge trim — steel */}
      <mesh position={[0, H + topThk / 2 + 0.003, 0]}>
        <boxGeometry args={[W + 0.01, 0.006, D + 0.01]} />
        <meshStandardMaterial color={LAB.steel} roughness={0.25} metalness={0.9} />
      </mesh>

      {/* Lower shelf */}
      <mesh position={[0, 0.30, 0]} castShadow receiveShadow>
        <boxGeometry args={[W - 0.06, 0.025, D - 0.1]} />
        <meshStandardMaterial color={LAB.benchBody} roughness={0.55} metalness={0.35} />
      </mesh>

      {/* 4 Legs */}
      {[
        [-W / 2 + 0.04,  D / 2 - 0.04],
        [ W / 2 - 0.04,  D / 2 - 0.04],
        [-W / 2 + 0.04, -D / 2 + 0.04],
        [ W / 2 - 0.04, -D / 2 + 0.04],
      ].map(([lx, lz], i) => (
        <mesh key={`leg-${i}`} position={[lx, H / 2, lz]} castShadow>
          <cylinderGeometry args={[legR, legR, H, 8]} />
          <meshStandardMaterial color={LAB.benchLeg} roughness={0.2} metalness={0.9} />
        </mesh>
      ))}

      {/* Horizontal stretcher rails (front + rear) */}
      <mesh position={[0, 0.30, D / 2 - 0.04]} castShadow>
        <boxGeometry args={[W - 0.08, 0.02, 0.02]} />
        <meshStandardMaterial color={LAB.benchLeg} roughness={0.25} metalness={0.85} />
      </mesh>
      <mesh position={[0, 0.30, -D / 2 + 0.04]} castShadow>
        <boxGeometry args={[W - 0.08, 0.02, 0.02]} />
        <meshStandardMaterial color={LAB.benchLeg} roughness={0.25} metalness={0.85} />
      </mesh>

      {/* Back upright rail */}
      <mesh position={[0, H + 0.35, -D / 2 + 0.025]} castShadow>
        <boxGeometry args={[W, 0.02, 0.02]} />
        <meshStandardMaterial color={LAB.steel} roughness={0.2} metalness={0.9} />
      </mesh>
    </group>
  );
}

// ═══ Monitor with Stand ═══════════════════════════════════════════════
function LabMonitor({ position }: { position: [number, number, number] }) {
  const [mx, my, mz] = position;
  return (
    <group position={[mx, my, mz]}>
      {/* Stand base */}
      <mesh castShadow receiveShadow>
        <boxGeometry args={[0.22, 0.02, 0.18]} />
        <meshStandardMaterial color={LAB.darkSteel} roughness={0.3} metalness={0.75} />
      </mesh>
      {/* Stand neck */}
      <mesh position={[0, 0.20, -0.05]} castShadow>
        <boxGeometry args={[0.04, 0.40, 0.03]} />
        <meshStandardMaterial color="#404550" roughness={0.3} metalness={0.7} />
      </mesh>
      {/* Screen housing */}
      <mesh position={[0, 0.42, -0.03]} castShadow>
        <boxGeometry args={[0.52, 0.30, 0.038]} />
        <meshStandardMaterial color={LAB.screenBody} roughness={0.4} metalness={0.6} />
      </mesh>
      {/* Screen bezel inner */}
      <mesh position={[0, 0.42, -0.012]}>
        <boxGeometry args={[0.48, 0.27, 0.005]} />
        <meshStandardMaterial color="#10141e" roughness={0.15} metalness={0.3} />
      </mesh>
      {/* Screen glow — lab OS / data display */}
      <mesh position={[0, 0.42, -0.009]}>
        <boxGeometry args={[0.44, 0.24, 0.001]} />
        <meshStandardMaterial
          color={LAB.screenGlow}
          emissive={LAB.screenGlow}
          emissiveIntensity={0.55}
          toneMapped={false}
          roughness={0.05}
        />
      </mesh>
      {/* Screen content — data grid lines */}
      {[-0.08, -0.02, 0.04, 0.10].map((y, i) => (
        <mesh key={`row-${i}`} position={[0, 0.42 + y, -0.008]}>
          <boxGeometry args={[0.38, 0.003, 0.001]} />
          <meshStandardMaterial
            color="#a8d4f0"
            emissive="#a8d4f0"
            emissiveIntensity={0.6}
            toneMapped={false}
          />
        </mesh>
      ))}
      {/* Power LED */}
      <mesh position={[0.22, 0.28, -0.010]}>
        <sphereGeometry args={[0.005, 8, 8]} />
        <meshStandardMaterial
          color={LAB.ledGreen}
          emissive={LAB.ledGreen}
          emissiveIntensity={1.2}
          toneMapped={false}
        />
      </mesh>
    </group>
  );
}

// ═══ Equipment Rack ═══════════════════════════════════════════════════
// A proper 19" style rack with rails and instrument panels
function EquipmentRack({ position }: { position: [number, number, number] }) {
  const [rx, ry, rz] = position;
  return (
    <group position={[rx, ry, rz]}>
      {/* Rack body */}
      <mesh castShadow receiveShadow>
        <boxGeometry args={[0.52, 1.30, 0.48]} />
        <meshStandardMaterial color={LAB.rack} roughness={0.55} metalness={0.4} />
      </mesh>
      {/* Rack front face */}
      <mesh position={[0, 0, 0.242]}>
        <boxGeometry args={[0.49, 1.27, 0.005]} />
        <meshStandardMaterial color="#a8adb8" roughness={0.45} metalness={0.5} />
      </mesh>
      {/* Rail strips */}
      {[-0.21, 0.21].map((x, i) => (
        <mesh key={`rail-${i}`} position={[x, 0, 0.244]}>
          <boxGeometry args={[0.025, 1.24, 0.004]} />
          <meshStandardMaterial color={LAB.darkSteel} roughness={0.25} metalness={0.8} />
        </mesh>
      ))}
      {/* Instrument panels */}
      {[-0.48, -0.26, -0.04, 0.18, 0.40].map((y, i) => (
        <group key={`panel-${i}`} position={[0, y, 0.246]}>
          {/* Panel body */}
          <mesh>
            <boxGeometry args={[0.44, 0.16, 0.012]} />
            <meshStandardMaterial color={LAB.rackPanel} roughness={0.35} metalness={0.65} />
          </mesh>
          {/* Panel LEDs */}
          {[0].map((_, li) => (
            <mesh key={`led-${li}`} position={[-0.18 + li * 0.06, 0.04, 0.009]}>
              <sphereGeometry args={[0.006, 8, 8]} />
              <meshStandardMaterial
                color={i % 2 === 0 ? LAB.ledGreen : LAB.ledAmber}
                emissive={i % 2 === 0 ? LAB.ledGreen : LAB.ledAmber}
                emissiveIntensity={1.0}
                toneMapped={false}
              />
            </mesh>
          ))}
          {/* Knob */}
          <mesh position={[0.14, 0, 0.012]}>
            <cylinderGeometry args={[0.012, 0.012, 0.018, 12]} />
            <meshStandardMaterial color={LAB.steel} roughness={0.2} metalness={0.9} />
          </mesh>
          {/* Display strip */}
          <mesh position={[-0.04, -0.02, 0.010]}>
            <boxGeometry args={[0.20, 0.06, 0.002]} />
            <meshStandardMaterial
              color="#1c2840"
              emissive="#3a6090"
              emissiveIntensity={0.4}
              toneMapped={false}
              roughness={0.05}
            />
          </mesh>
        </group>
      ))}
    </group>
  );
}

// ═══ Storage Cabinet ═══════════════════════════════════════════════════
function StorageCabinet({ position }: { position: [number, number, number] }) {
  const [cx, cy, cz] = position;
  return (
    <group position={[cx, cy, cz]}>
      {/* Cabinet body */}
      <mesh castShadow receiveShadow>
        <boxGeometry args={[0.55, 1.80, 0.40]} />
        <meshStandardMaterial color={LAB.cabinet} roughness={0.75} metalness={0.1} />
      </mesh>
      {/* Door left */}
      <mesh position={[-0.14, 0.1, 0.203]}>
        <boxGeometry args={[0.255, 1.55, 0.012]} />
        <meshStandardMaterial color={LAB.cabinetDoor} roughness={0.65} metalness={0.12} />
      </mesh>
      {/* Door right */}
      <mesh position={[0.14, 0.1, 0.203]}>
        <boxGeometry args={[0.255, 1.55, 0.012]} />
        <meshStandardMaterial color={LAB.cabinetDoor} roughness={0.65} metalness={0.12} />
      </mesh>
      {/* Handles */}
      {[-0.04, 0.04].map((x, i) => (
        <mesh key={`handle-${i}`} position={[x, 0.25, 0.218]} castShadow>
          <cylinderGeometry args={[0.007, 0.007, 0.10, 8]} />
          <meshStandardMaterial color={LAB.steel} roughness={0.15} metalness={0.95} />
        </mesh>
      ))}
      {/* Top shelf display — small LED panel */}
      <mesh position={[0, 0.87, 0.204]}>
        <boxGeometry args={[0.30, 0.06, 0.006]} />
        <meshStandardMaterial
          color="#1a2030"
          emissive="#3060a0"
          emissiveIntensity={0.35}
          toneMapped={false}
        />
      </mesh>
      {/* Plinth / base */}
      <mesh position={[0, -0.91, 0]} receiveShadow>
        <boxGeometry args={[0.57, 0.02, 0.42]} />
        <meshStandardMaterial color="#b0b4bc" roughness={0.6} metalness={0.3} />
      </mesh>
    </group>
  );
}

// ═══ Small Lab Props ═══════════════════════════════════════════════════
function LabProps() {
  return (
    <group>
      {/* ── Parts tray (safety orange) on the bench ── */}
      <group position={[0.45, 0.86, -0.90]}>
        {/* Tray base */}
        <mesh castShadow receiveShadow>
          <boxGeometry args={[0.22, 0.04, 0.15]} />
          <meshStandardMaterial color={LAB.trayOrange} roughness={0.7} metalness={0.1} />
        </mesh>
        {/* Tray rim */}
        {[
          { pos: [0, 0.025, -0.073] as [number,number,number], size: [0.22, 0.01, 0.006] as [number,number,number] },
          { pos: [0, 0.025,  0.073] as [number,number,number], size: [0.22, 0.01, 0.006] as [number,number,number] },
          { pos: [-0.11, 0.025, 0] as [number,number,number], size: [0.006, 0.01, 0.15] as [number,number,number] },
          { pos: [ 0.11, 0.025, 0] as [number,number,number], size: [0.006, 0.01, 0.15] as [number,number,number] },
        ].map((r, i) => (
          <mesh key={`rim-${i}`} position={r.pos}>
            <boxGeometry args={r.size} />
            <meshStandardMaterial color={LAB.trayOrange} roughness={0.7} metalness={0.1} />
          </mesh>
        ))}
        {/* Small parts (grey cubes in tray) */}
        {[[-0.06, 0.035, -0.03], [0, 0.030, 0.02], [0.06, 0.035, -0.01]].map((p, i) => (
          <mesh key={`part-${i}`} position={p as [number,number,number]} castShadow>
            <boxGeometry args={[0.03, 0.025, 0.025]} />
            <meshStandardMaterial color={LAB.steel} roughness={0.4} metalness={0.7} />
          </mesh>
        ))}
      </group>

      {/* ── Multimeter on bench ── */}
      <group position={[-0.55, 0.865, -1.02]}>
        <mesh castShadow>
          <boxGeometry args={[0.09, 0.18, 0.04]} />
          <meshStandardMaterial color={LAB.whitePlastic} roughness={0.65} metalness={0.05} />
        </mesh>
        {/* Screen */}
        <mesh position={[0, 0.04, 0.022]}>
          <boxGeometry args={[0.065, 0.06, 0.004]} />
          <meshStandardMaterial
            color="#0d2010"
            emissive="#20a030"
            emissiveIntensity={0.6}
            toneMapped={false}
          />
        </mesh>
        {/* Probe cables */}
        <mesh position={[0, -0.06, 0.023]}>
          <boxGeometry args={[0.04, 0.01, 0.005]} />
          <meshStandardMaterial color="#c84040" roughness={0.7} />
        </mesh>
      </group>

      {/* ── Cable spool on lower shelf ── */}
      <group position={[-0.40, 0.32, -1.02]}>
        <mesh castShadow rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.07, 0.07, 0.06, 20]} />
          <meshStandardMaterial color={LAB.whitePlastic} roughness={0.5} metalness={0.1} />
        </mesh>
        {/* Cable wrap */}
        <mesh castShadow rotation={[0, 0, Math.PI / 2]}>
          <torusGeometry args={[0.05, 0.016, 8, 24]} />
          <meshStandardMaterial color={LAB.cableGrey} roughness={0.6} metalness={0.2} />
        </mesh>
      </group>

      {/* ── Notebook / logbook ── */}
      <mesh position={[0.20, 0.862, -1.05]} rotation={[0, -0.15, 0]} castShadow>
        <boxGeometry args={[0.18, 0.008, 0.24]} />
        <meshStandardMaterial color="#3a5c8a" roughness={0.85} metalness={0.0} />
      </mesh>

      {/* ── Pencil on notebook ── */}
      <mesh position={[0.22, 0.868, -0.97]} rotation={[0, 0.3, 0]} castShadow>
        <cylinderGeometry args={[0.006, 0.006, 0.18, 6]} />
        <meshStandardMaterial color="#e8c040" roughness={0.6} metalness={0.0} />
      </mesh>
    </group>
  );
}

function SceneContent() {
  return (
    <>
      {/* Camera: slightly above and to the side, looking at torso+arm */}
      <PerspectiveCamera makeDefault position={[0.55, 0.68, 1.10]} fov={38} />
      {/* No autoRotate — user is free to orbit the scene */}
      <OrbitControls
        enableDamping
        dampingFactor={0.08}
        minDistance={0.4}
        maxDistance={4.5}
        target={[0.10, 0.52, 0]}
        maxPolarAngle={Math.PI / 2 - 0.02}
      />

      <SceneLighting />

      {/* ═══ FLOOR — warm off-white vinyl tile ═══ */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]} receiveShadow>
        <planeGeometry args={[20, 20]} />
        <meshStandardMaterial color={LAB.floor} roughness={0.85} metalness={0.01} />
      </mesh>

      {/* ═══ GRID — warm subtle tile lines ═══ */}
      <Grid
        args={[20, 20]}
        position={[0, 0.001, 0]}
        cellSize={0.5}
        cellThickness={0.3}
        cellColor={LAB.floorGrid}
        sectionSize={2.0}
        sectionThickness={0.5}
        sectionColor={LAB.floorGridSec}
        fadeDistance={10}
        fadeStrength={2.5}
        infiniteGrid={false}
      />

      {/* ═══ LAB STRUCTURE ═══ */}
      <LabWalls />

      {/* ═══ WORKBENCH — behind and left ═══ */}
      <Workbench position={[-0.10, 0, -1.08]} />

      {/* ═══ MONITOR on the workbench ═══ */}
      <LabMonitor position={[-0.30, 0.86, -1.18]} />

      {/* ═══ EQUIPMENT RACK — far left against wall ═══ */}
      <EquipmentRack position={[-2.0, 0.65, -1.0]} />

      {/* ═══ STORAGE CABINET — right side against wall ═══ */}
      <StorageCabinet position={[1.60, 0.90, -1.90]} />

      {/* ═══ SMALL LAB PROPS ═══ */}
      <LabProps />

      {/* ═══ TORSO BODY ═══ */}
      <TorsoMount />

      {/* ═══ BIONIC ARM — mounts from shoulder ═══ */}
      <group
        position={[0.21, 0.74, 0]}
        rotation={[0.22, 0.18, -Math.PI / 2]}
      >
        <BionicHand />
      </group>
    </>
  );
}

export default function HandViewer() {
  return (
    <Canvas
      shadows
      gl={{
        antialias: true,
        toneMapping: THREE.ACESFilmicToneMapping,
        toneMappingExposure: 0.92,
      }}
      style={{ width: "100%", height: "100%" }}
      onCreated={({ gl, scene }) => {
        // Soft warm sky blue — like a lab with high windows
        gl.setClearColor(new THREE.Color("#c8d8e8"));
        scene.fog = new THREE.FogExp2("#ccd8e4", 0.055);
      }}
    >
      <SceneContent />
    </Canvas>
  );
}
