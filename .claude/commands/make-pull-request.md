## プルリクエスト作成ガイド

### 事前チェック
- [ ] 最新のmainブランチから作業ブランチを作成済み
- [ ] 関連するissueがあれば番号を確認
- [ ] 変更内容が単一の責任範囲に収まっている

### Git コミット規則

#### Pre-commit フック
- コミット時にpre-commitエラーが発生した場合、必ずすべてのエラーを修正してから再コミットすること
- pre-commitフックによる自動修正がある場合は、その変更を含めてコミットすること

#### Conventional Commits
- コミットメッセージは必ずConventional Commits形式に従うこと
  - 形式: `<type>(<scope>): <subject>`
  - 例: `feat(auth): add login functionality`
  - 例: `fix(api): resolve null pointer exception`
  - 例: `docs(readme): update installation instructions`

### PR作成時の規則
- PRタイトルもConventional Commits形式に従うこと
- PR本文には以下を含めること：
  - 変更内容の概要
  - 関連するissue番号（`Closes #123`など）
  - テスト方法（必要に応じて）
  - スクリーンショット（UI変更の場合）

## 利用可能なタイプ
- `feat`: 新機能
- `fix`: バグ修正
- `docs`: ドキュメントの変更
- `style`: フォーマット、セミコロンの欠落など（コードの動作に影響しない変更）
- `refactor`: リファクタリング（バグ修正でも機能追加でもない）
- `test`: テストの追加・修正
- `chore`: ビルドプロセスやツールの変更
- `ci`: CI/CDの設定変更
- `perf`: パフォーマンス改善
- `revert`: 以前のコミットの取り消し

## PRテンプレート例
```
## 概要
<!-- 変更内容を簡潔に説明 -->

## 関連Issue
<!-- Closes #123 -->

## 変更内容
- [ ] 項目1
- [ ] 項目2

## テスト方法
<!-- 動作確認の手順があれば記載 -->

## その他
<!-- 追加の注意事項があれば記載 -->
```

## PR作成後の監視・対処

### CI/CDパイプラインの監視
PR作成直後からgithub actionsを継続的に監視すること：

#### CI失敗時の対処手順
1. **即座に失敗原因を確認**
   - CIログを詳細に確認
   - 失敗したテストやビルドステップを特定

2. **修正実施**
   - テストローカルで実行して修正内容を確認
   - 修正後に再度プッシュ

3. **再監視**
   - 修正後のCIが正常に完了するまで監視継続