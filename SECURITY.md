# Security Policy for ApexPathAI

## Supported Releases
Security updates and critical patches are actively provided for the following releases:

| Release Series | Supported |
| :--- | :--- |
| `2.0.x` | Yes (Current Stable) |
| `< 2.0.0` | No |

## Reporting Security Vulnerabilities
If you discover a potential security flaw, vulnerability, or safety risk in ApexPathAI (such as unauthenticated remote control, denial-of-service, or path traversal), please notify the maintainers privately rather than filing a public issue.

- **Primary Contact:** Submit a confidential report via GitHub Private Vulnerability Reporting on the repository, or email `security@apexpath.ai`.
- **Response Commitment:** Our maintenance team aims to acknowledge submissions within 48 hours and provide an initial impact assessment within 5 business days.
- **Coordination:** We request that reporters maintain confidentiality until an official security patch or advisory has been released.

## Autonomous Systems Deployment Guidance
When deploying ApexPathAI in physical test tracks or hardware-in-the-loop environments:
1. **Network Binding:** By default, run the telemetry server bound to loopback `127.0.0.1`. Do not expose the service on public networks without an authenticating reverse proxy (e.g. Nginx with TLS and token verification).
2. **Emergency Stop (E-STOP):** Treat all control endpoints (`/api/v1/safety/estop`) as safety-critical interfaces. Physical vehicles must always maintain an independent hardware-level E-STOP circuit that does not rely on software or network links.
3. **CORS Configuration:** In production deployments, configure allowed web origins explicitly rather than using wildcard access.
