pub fn generate_login_html() -> String {
    r#"<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>HORUS Monitor - Login</title>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700;900&display=swap');

        * { margin: 0; padding: 0; box-sizing: border-box; }

        body {
            font-family: 'Orbitron', monospace;
            background: #0a0a0f;
            min-height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            padding: 20px;
            position: relative;
            overflow: hidden;
        }

        body::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background-image:
                linear-gradient(rgba(0, 255, 255, 0.1) 1px, transparent 1px),
                linear-gradient(90deg, rgba(0, 255, 255, 0.1) 1px, transparent 1px);
            background-size: 50px 50px;
            animation: gridMove 20s linear infinite;
            z-index: 0;
        }

        @keyframes gridMove {
            0% { transform: translate(0, 0); }
            100% { transform: translate(50px, 50px); }
        }

        body::after {
            content: '';
            position: absolute;
            width: 500px;
            height: 500px;
            background: radial-gradient(circle, rgba(0, 255, 136, 0.2), transparent);
            border-radius: 50%;
            top: -250px;
            right: -250px;
            animation: pulse 4s ease-in-out infinite;
            z-index: 0;
        }

        @keyframes pulse {
            0%, 100% { opacity: 0.5; transform: scale(1); }
            50% { opacity: 0.8; transform: scale(1.1); }
        }

        .login-container {
            background: rgba(15, 15, 25, 0.9);
            padding: 40px;
            border-radius: 20px;
            border: 2px solid #00ffff;
            box-shadow:
                0 0 20px rgba(0, 255, 255, 0.5),
                0 0 40px rgba(0, 255, 136, 0.3),
                inset 0 0 60px rgba(0, 255, 255, 0.1);
            max-width: 400px;
            width: 100%;
            position: relative;
            z-index: 1;
            backdrop-filter: blur(10px);
        }

        .logo {
            text-align: center;
            margin-bottom: 30px;
        }

        .logo h1 {
            color: #00ffff;
            font-size: 42px;
            margin-bottom: 8px;
            text-shadow:
                0 0 10px #00ffff,
                0 0 20px #00ffff,
                0 0 30px #00ffff;
            font-weight: 900;
            letter-spacing: 4px;
            animation: glitch 3s infinite;
        }

        @keyframes glitch {
            0%, 90%, 100% { transform: translate(0); }
            92% { transform: translate(-2px, 2px); }
            94% { transform: translate(2px, -2px); }
            96% { transform: translate(-2px, -2px); }
            98% { transform: translate(2px, 2px); }
        }

        .logo p {
            color: #00ff88;
            font-size: 14px;
            text-transform: uppercase;
            letter-spacing: 3px;
            text-shadow: 0 0 10px #00ff88;
        }

        .form-group {
            margin-bottom: 20px;
        }

        label {
            display: block;
            color: #00ffff;
            font-weight: 600;
            margin-bottom: 8px;
            font-size: 14px;
            text-transform: uppercase;
            letter-spacing: 2px;
        }

        input[type="password"] {
            width: 100%;
            padding: 12px 16px;
            background: rgba(0, 0, 0, 0.5);
            border: 2px solid #00ffff;
            border-radius: 8px;
            font-size: 16px;
            color: #fff;
            font-family: 'Orbitron', monospace;
            transition: all 0.3s;
        }

        input[type="password"]:focus {
            outline: none;
            border-color: #00ff88;
            box-shadow:
                0 0 10px #00ff88,
                inset 0 0 10px rgba(0, 255, 136, 0.2);
        }

        .btn {
            width: 100%;
            padding: 14px;
            background: linear-gradient(135deg, #00ffff 0%, #00ff88 100%);
            color: #0a0a0f;
            border: none;
            border-radius: 8px;
            font-size: 16px;
            font-weight: 700;
            cursor: pointer;
            transition: all 0.3s;
            text-transform: uppercase;
            letter-spacing: 2px;
            font-family: 'Orbitron', monospace;
            box-shadow: 0 0 20px rgba(0, 255, 255, 0.5);
        }

        .btn:hover {
            transform: translateY(-2px);
            box-shadow:
                0 0 30px #00ffff,
                0 0 40px #00ff88;
        }

        .btn:active {
            transform: translateY(0);
        }

        .error {
            background: rgba(255, 0, 100, 0.2);
            color: #ff0066;
            padding: 12px;
            border-radius: 8px;
            border: 1px solid #ff0066;
            margin-bottom: 20px;
            font-size: 14px;
            display: none;
            text-shadow: 0 0 5px #ff0066;
        }

        .error.show {
            display: block;
            animation: errorPulse 0.5s;
        }

        @keyframes errorPulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }

        .info {
            text-align: center;
            color: #00ff88;
            font-size: 11px;
            margin-top: 20px;
            text-transform: uppercase;
            letter-spacing: 1px;
            text-shadow: 0 0 5px #00ff88;
        }
    
        /* Mobile Responsive Styles */
        @media (max-width: 768px) {
            body {
                padding: 10px;
            }

            body::before {
                background-size: 30px 30px;
            }

            body::after {
                width: 300px;
                height: 300px;
                top: -150px;
                right: -150px;
            }

            .login-container {
                padding: 30px 20px;
                max-width: 100%;
                border-radius: 16px;
            }

            .logo h1 {
                font-size: 36px;
                letter-spacing: 3px;
            }

            .logo p {
                font-size: 12px;
                letter-spacing: 2px;
            }

            label {
                font-size: 12px;
                letter-spacing: 1.5px;
            }

            input[type="password"] {
                padding: 14px 16px;
                font-size: 16px;
            }

            .btn {
                padding: 16px;
                font-size: 15px;
            }

            .info {
                font-size: 10px;
            }
        }

        @media (max-width: 480px) {
            .login-container {
                padding: 25px 15px;
            }

            .logo h1 {
                font-size: 32px;
            }
        }

    </style>
</head>
<body>
    <div class="login-container">
        <div class="logo">
            <h1>HORUS</h1>
            <p>Robotics Monitor</p>
        </div>

        <div id="error" class="error"></div>

        <form id="loginForm">
            <div class="form-group">
                <label for="password">Password</label>
                <input type="password" id="password" name="password" required autofocus>
            </div>

            <button type="submit" class="btn">Login</button>
        </form>

        <div class="info">
            Secure Connection Established
        </div>
    </div>

    <script>
        const form = document.getElementById('loginForm');
        const errorDiv = document.getElementById('error');

        form.addEventListener('submit', async (e) => {
            e.preventDefault();
            errorDiv.classList.remove('show');

            const password = document.getElementById('password').value;

            try {
                const response = await fetch('/api/login', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({ password })
                });

                const data = await response.json();

                if (data.success) {
                    window.location.reload();
                } else {
                    errorDiv.textContent = data.error || 'Invalid password';
                    errorDiv.classList.add('show');
                    document.getElementById('password').value = '';
                    document.getElementById('password').focus();
                }
            } catch (error) {
                errorDiv.textContent = 'Network error. Please try again.';
                errorDiv.classList.add('show');
            }
        });
    </script>
</body>
</html>"#.to_string()
}

