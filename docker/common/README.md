# docker/common

dev/sim/driver 3コンテナで共有する対話シェル(zsh)設定。各Dockerfileから
`~/.zshrc`・`~/.config/starship.toml` としてCOPYされる(このディレクトリ単体では実行しない)。

## なぜこの構成か

ホストの `~/.zshrc`(dotfiles管理、prezto + Powerlevel10k + fzf)と同じ体験をコンテナに
持ち込むと、prezto/p10kのgit clone・instant prompt用キャッシュ・Nerd Font前提のアイコンなど、
コンテナのビルド時間や環境差(GUI無し・フォント無し)に対して重すぎる/そのまま動かない部分が多い。
そのため「予測入力の便利さ」「見やすいプロンプト」という体験の中身だけを、軽量な単体パッケージ・
単一バイナリで再現する方針にした。

| ホストの構成要素 | コンテナでの代替 | 理由 |
|---|---|---|
| prezto | `zsh-autosuggestions` + `zsh-syntax-highlighting`(apt) + 標準`compinit`のメニュー選択補完 | frameworkのgit clone不要。apt一発で入り、機能面(予測入力・色付き補完)は同等 |
| Powerlevel10k | [starship](https://starship.rs)(公式インストールスクリプトで導入する単一静的バイナリ) | instant prompt等の高度なキャッシュ機構が不要。`docker/common/starship.toml`はNerd Font不要のASCII安全構成(素のUnicode矢印`❯`+絵文字のみ)にしてあり、フォント未設定のターミナルでも文字化けしない |
| `aliases.zsh`(ll/la/l等) | `docker/common/zshrc`にそのまま移植 | 見た目の使用感を揃える |
| なし(コンテナ固有の追加) | `cb`/`cbp`エイリアス、`ROS_DISTRO`に応じた`setup.zsh`の自動source | ROS2作業前提のコンテナならではの追加 |

## 含まれるファイル

- `zshrc`: 共通の`~/.zshrc`本体。履歴・補完・予測入力・starship初期化・エイリアスに加え、
  `$ROS_DISTRO`(ベースイメージ側で設定済み)に応じて`/opt/ros/$ROS_DISTRO/setup.zsh`を自動source
- `starship.toml`: プロンプト設定。ディレクトリ・gitブランチ・`$ROS_DISTRO`表示・成功/失敗記号のみの最小構成

各コンテナ固有のワークスペースsource(`~/ros2_ws/install/setup.zsh`等)は、この共通ファイルの
後ろに各Dockerfileが`RUN echo '...' >> ~/.zshrc`で追記する形にしてあり、`docker/common/zshrc`
自体はどのコンテナにも共通の内容に保っている。

## 使い方

利用者側で意識することは基本無い。各コンテナの対話シェルは`zsh`が既定(`docker compose exec <service> zsh`)で、
起動すると自動的にROS2・ワークスペースがsource済みの状態になる。`bash`も引き続き使用可能
(`.bashrc`側のROS2 source設定は変更していない)。

driverコンテナのみ、CycloneDDS/RMW設定(`docker/driver/setup_dds.sh`)がbash/zsh両対応になっており、
`$ZSH_VERSION`の有無でROS2 setupスクリプトの`.bash`/`.zsh`を切り替えている(詳細はそのファイル参照)。
