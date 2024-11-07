適当に書いてください。
構造はこんなイメージです。
だいたいあってればいいので気にしないでください。
airhockey_project/
├── README.md
├── requirements.txt
├── setup.py
├── data/
│   ├── raw/
│   ├── processed/
│   └── models/
├── src/
│   ├── __init__.py
│   ├── data/
│   │   ├── __init__.py
│   │   ├── load_data.py
│   │   └── preprocess.py
│   ├── models/
│   │   ├── __init__.py
│   │   ├── train.py
│   │   └── evaluate.py
│   ├── visualization/
│   │   ├── __init__.py
│   │   └── visualize.py
│   ├── reinforcement_learning/
│   │   ├── __init__.py
│   │   ├── train_rl.py
│   │   └── evaluate_rl.py
│   ├── imitation_learning/
│   │   ├── __init__.py
│   │   ├── train_il.py
│   │   └── evaluate_il.py
│   └── simulation/
│       ├── __init__.py
│       ├── gazebo/
│       │   ├── models/
│       │   ├── worlds/
│       │   └── launch/
│       ├── start_gazebo.py
│       └── control_robot.py
└── scripts/
    ├── run_training.sh
    └── run_evaluation.sh