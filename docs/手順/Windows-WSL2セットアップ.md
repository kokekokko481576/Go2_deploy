# Windows(WSL2)セットアップ手順 — 「これをやるだけ」版 (Issue #28)

Windows機で本リポジトリのsim+dev環境を動かすための手順書。#28の方針どおり
**WSL2ネイティブ運用**(WSL2内のUbuntuにDocker Engineを直接入れる。Docker Desktopは使わない)。
上から順にやれば動く想定。詰まったら「トラブルシュート」を見て、直らなければ
詰まり方をissue #28(GUIは#29、DDS疎通は#30)に記録して改善する。

想定: Windows 11 (Windows 10 22H2でも可)、AMD/Intel/NVIDIAいずれかのGPU。

**事前に用意するもの(前提)**:

- **ストレージ空き容量: 30〜40GB を推奨**(最低25GB)。内訳は Docker イメージ実測で
  sim(`go2-sim:jazzy`)約7.6GB + dev(`arbeit-ros2:humble`)約6.0GB に、WSL2/Ubuntu/Docker基盤
  約3.5GB、初回ビルド中の中間レイヤ・キャッシュが一時的に+10〜15GB。ビルド完了後の実効は
  約18GBだが、ビルド中のピークで詰まると初回ビルド(15〜40分)が失敗するため余裕を持たせる。
  ビルド後に一時領域を回収したければ `docker builder prune` でキャッシュ分を空けられる。
  (実機driver 1.6GB は #31 の段階まで不要)
- **メモリ(RAM): 16GB 以上を推奨**。`.wslconfig` で `memory=12GB` をWSL2に割り当てる(ビルドとGazeboで食う)。

## 最短手順(スクリプト3本、これをやるだけ)

1. **GPUドライバをベンダー公式から最新に更新**(AMDならAdrenalin。ここだけ手動)
2. **PowerShell(管理者)** で:

   ```powershell
   irm https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/windows-setup.ps1 | iex
   ```

   <details open><summary>WSL新規導入なら再起動→Ubuntuの初期ユーザ作成まで済ませる</summary>
      
      WSLを初めてインストールした環境では、スクリプト実行後にWindowsの再起動が必須である。
      
      再起動後、自動でUbuntuのターミナルが開くため、画面の指示に従って初期ユーザー名とパスワードの設定を済ませること。
   
      万が一、再起動後にUbuntuが起動しない、または「ディストリビューションはありません」という旨のエラーが出た場合は、再度PowerShell(管理者)を開き、以下のコマンドを手動で実行してUbuntuをインストールすること。
      
      ```PowerShell
      wsl --install
      ```
      こんなかんじ
      <img width="1391" height="1117" alt="image" src="https://github.com/user-attachments/assets/9cd0bc4f-cab8-47e0-b6a2-a201f23ae79b" />

      wslを入れた後なら、PowerShellを起動して

      ```PowerShell
      wsl
      ```
      と打ったらいつでも起動できるようになる
   </details>
   
   
   

4. **Ubuntu(WSL2)のターミナル** で:

   ```bash
   curl -fsSL https://raw.githubusercontent.com/kokekokko481576/Go2_deploy/main/scripts/wsl2-setup.sh | bash
   ```

   [OK]/[NG]の自己診断つき。NGが出たら指示に従って解消→再実行(何度でも安全)
5. **ターミナルを開き直して**:

   ```bash
   cd ~/Go2_deploy && ./scripts/first-run.sh
   ```

   ビルド(初回15〜40分)→起動→DDS疎通チェックまで自動。最後に「全チェック通過」と
   Gazebo GUIウィンドウ(目視)が確認できたら完了。詰まったら[NG]メッセージ内の
   issue番号(#28/#29/#30)へ記録

以降の節は、スクリプトが中でやっていることの説明と、詰まったとき用の詳細。

## 1. Windows側の準備(windows-setup.ps1がやること)

1. **GPUドライバをベンダー公式から最新に更新**(AMDなら Adrenalin)。
   WSL2のGPU仮想化(/dev/dxg)はWindows側のWDDMドライバが担うため、これが古いと
   コンテナ内のGazebo描画がソフトウェアレンダリングに落ちる
2. PowerShell(管理者)で WSL2 + Ubuntu 22.04 を導入:

   ```powershell
   wsl --install -d Ubuntu-22.04
   wsl --update
   ```

   既にWSLがある人は `wsl --version` でWSLg同梱の新しめ(1.0+)かを確認
3. (推奨) `%UserProfile%\.wslconfig` を作りメモリ上限を明示(ビルドとGazeboで食うため):

   ```ini
   [wsl2]
   memory=12GB
   ```

   ※実機Go2との接続(#31)をやる段になったら `networkingMode=mirrored` が要るが、
   sim+devだけなら**不要**。今は書かない

## 2. WSL2のUbuntu側の準備(wsl2-setup.shがやること)

Ubuntuのターミナル(wsl起動)で:

1. **Docker Engineを入れる**(Docker Desktopではない点に注意):

   ```bash
   sudo apt-get update && sudo apt-get install -y ca-certificates curl
   sudo install -m 0755 -d /etc/apt/keyrings
   sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
     https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) stable" | \
     sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
   sudo apt-get update && sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
   sudo usermod -aG docker $USER
   ```

   一度ターミナルを閉じて開き直し、`docker run --rm hello-world` が通ればOK。
   最近のWSL2 UbuntuはsystemdがデフォルトON(`systemctl status docker`で確認できる)。
   もし `System has not been booted with systemd` と出たら `/etc/wsl.conf` に
   `[boot]\nsystemd=true` を書いて PowerShellから `wsl --shutdown` → 再起動
2. **GPUとGUIの事前チェック**(コンテナに入る前にWSL2自体で確認する):

   ```bash
   ls /dev/dxg          # ある → GPU仮想化OK(Windows側ドライバが新しい)
   ls /dev/dri          # card0/renderD128 がある → コンテナへのGPU渡し(compose既定)がそのまま使える
   ls /tmp/.X11-unix    # X0 がある → WSLgのX11ソケット。GUIはこれ経由で出る(xhost不要のことが多い)
   echo $DISPLAY        # :0 など値が入っていること
   ```

   `/dev/dri`が無い場合はWindows側GPUドライバ更新→`wsl --shutdown`→再試行。
   それでも無ければトラブルシュート参照(ソフトレンダリングで進める手はある)

## 3. リポジトリ取得(wsl2-setup.shがやること)

1. **cloneは必ずWSL2側のファイルシステム(`~/`配下)に置く**。`/mnt/c/...`(Windows側)に
   置くとビルドが激遅になり、改行コード問題も踏む(ルートREADMEの注意と同じ。#32)
2. submoduleがSSH URL(`git@github.com:`)参照のため、SSH鍵を作っていない人は
   HTTPSへの読み替えを先に設定してからcloneする:

   ```bash
   git config --global url."https://github.com/".insteadOf git@github.com:
   git clone --recurse-submodules https://github.com/kokekokko481576/Go2_deploy.git ~/Go2_deploy
   ```

   (chapter1はupdate=noneなので取得されなくて正常)

## 4. ビルドと起動(first-run.shがやること)

```bash
cd ~/Go2_deploy/docker/sim
docker compose build sim     # 初回15〜30分
docker compose up -d
docker logs -f go2-sim       # Gazeboが起動し、GazeboのGUIウィンドウがWindows側に出れば #29 クリア
```

別ターミナルで:

```bash
cd ~/Go2_deploy/docker
docker compose build && docker compose up -d
docker exec -it arbeit-ros2 zsh
ros2 topic list              # /robot1/... が見えれば sim⇔dev のDDS疎通OK = #30 クリア
ros2 topic echo /robot1/odometry/filtered --once   # 実データが届くことまで見る
```

そのまま経路追従の動作確認(`ros2_ws/src/go2_path_following/README.md`の
「使い方(フェーズB)」)まで通れば、Ubuntu環境と等価に使える状態。

## 5. 確認結果の記録

- Gazebo GUI表示の可否・見た目の異常 → #29 へ
- `ros2 topic list`のコンテナ間疎通・topic echoの実データ到達 → #30 へ
- Gazeboのリアルタイム係数(`docker/sim/README.md`の動作確認と同条件で) → #28 へ
- 実機Go2との有線接続はこの手順の対象外(**#31**。mirrored networking +
  Hyper-V firewall設定が別途必要になる見込み)

## トラブルシュート

- **Gazeboが真っ黒/激重**: コンテナ内でGPUが使えていない可能性。
  `docker exec -it go2-sim bash -c "apt list --installed 2>/dev/null | grep mesa-utils || apt-get install -y mesa-utils; glxinfo -B"`
  で `renderer` に **D3D12 (GPU名)** が出ればGPU有効、`llvmpipe` ならソフトレンダリング。
  ソフトレンダリングでも動作はする(遅いだけ)ので、検証を先に進めて構わない
- **`/dev/dri`が無くて `docker compose up` が失敗する**(`error gathering device
  information while adding custom device "/dev/dri"`): compose.yamlの
  `devices: /dev/dri` が原因。Windows側ドライバ更新で生えるのが本筋(GPUドライバ更新後、
  PowerShellで `wsl --shutdown` → 再起動 → `ls /dev/dri` で確認)。
  それでも無理なら、GPU渡しを外してソフトレンダリングで起動する回避策テンプレートを
  コピーして有効化する(コピー先はgitignore対象。既定のGPUパススルー構成には影響しない):

  ```bash
  cp docker/compose.override.yaml.example docker/compose.override.yaml
  cp docker/sim/compose.override.yaml.example docker/sim/compose.override.yaml
  ```

  `/dev/dri`が後で生えたら、コピーした2つの `compose.override.yaml` を削除すれば
  既定のGPUパススルー構成に戻る
- **GUIウィンドウが出ない**: `echo $DISPLAY`(WSL2側)が空ならWSLgが動いていない。
  `wsl --update` → `wsl --shutdown` → 再起動。xhostは通常不要だが、
  権限エラーが出たら `xhost +local:` を試す
- **apt/gitが遅い・変**: cloneが `/mnt/c` 配下にないか確認(§3-1)
- **ros2 topic listで何も見えない**: 両コンテナが同じWSL2内で動いているか
  (`docker ps`)、`ROS_DOMAIN_ID=0`のままかを確認。Windows版Dockerの
  host networkingはWSL2内では「WSL2カーネルのホストネットワーキング」なので
  Ubuntuと同条件のはず(ここが崩れていたら#30に詳細を記録)
