# Go2_deploy プロジェクトルール

## コミット規約

- コミットメッセージは**和文セマンティック形式**で書く
  - 形式: `<type>: <絵文字> <#issue番号> <和文の要約>`
  - 例: `docs: ✨ #1 dockerの要件定義を書いた`、`feat: 🐳 #1 Docker環境を追加`
  - type は conventional commits に準拠: feat / fix / docs / chore / refactor / test
  - 絵文字は gitmoji 準拠（✨新機能 🐛バグ修正 📝ドキュメント 🔧設定 ➕依存追加 🐳Docker など）
  - 対応する issue 番号を絵文字の直後に付ける
- `Co-Authored-By` 等のトレーラーは**付けない**
- 内容の異なる変更は分けてコミットする
- issueに関わる説明はissueにコメントする
- クローズできるissueを見つけたらユーザーに確認の上、closeする

## プロジェクト構成

- `docs/` — 計画ドキュメント（自己位置推定・経路生成・経路追従の3計画と統合作業計画）
- `docker/` — ROS2 Humble 開発環境（Nav2・robot_localization 等入り）
- `ros2_ws/` — colcon ワークスペース（build/install/log は gitignore 済み）
- `chapter1/` — AI-Robot-Book 教材のサブモジュール（update=none, ignore=all。触らない）
