# CycloneDDS/RMW設定(entrypoint.sh・対話シェルの.bashrc/.zshrc全てからsourceされる共通処理)。
# NIC名は要件定義R2どおりイメージに焼き込まず環境変数(GO2_NIC)から注入する。
# 実機なし(このPCでの動作確認)の場合は既定の "lo" のままでよい。
# 実機Go2に有線LANで接続する場合は GO2_NIC=enp3s0 のように環境変数で指定する。
NIC="${GO2_NIC:-lo}"

# loはmulticastフラグが立っていないことが多く(本機のloも同様)、CycloneDDSの既定である
# マルチキャストSPDP探索が機能しない。実機なしでの動作確認(NIC未指定=lo)時のみ
# ユニキャスト探索(Peer指定)に切り替える。実機NIC使用時はmulticastが使える前提のためlo限定。
if [ "${NIC}" = "lo" ]; then
    # ParticipantIndexを明示的なポート番号に固定(auto)しないと、Peer指定側が
    # 参加者のユニキャストポートを規定の計算式で当てられず発見できない(none=OSまかせのポートだと発見不可)。
    DISCOVERY_XML='<AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery>'
else
    DISCOVERY_XML='</General>'
fi

cat > /tmp/cyclonedds.xml <<EOF
<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="${NIC}" priority="default" multicast="default" />
</Interfaces>${DISCOVERY_XML}</Domain></CycloneDDS>
EOF

export CYCLONEDDS_URI="file:///tmp/cyclonedds.xml"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# zshrcからもsourceされるため、ROS2のセットアップスクリプトはシェルに応じて.bash/.zshを切り替える
# (setup.bashはBASH_SOURCE等bash固有の構文を含み、zshからsourceすると失敗するため)
if [ -n "$ZSH_VERSION" ]; then
    source /opt/ros/humble/setup.zsh
    source /root/ws/install/setup.zsh
else
    source /opt/ros/humble/setup.bash
    source /root/ws/install/setup.bash
fi
